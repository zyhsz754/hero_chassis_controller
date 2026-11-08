#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <cmath>

namespace hero_chassis_controller {
bool HeroChassisController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    std::vector<std::string> joint_names;
    controller_nh.getParam("joints", joint_names);
    for (const auto& name : joint_names) {
        joints_.push_back(hw->getHandle(name));
        control_toolbox::Pid pid;
        pid.init(controller_nh);
        pids_.push_back(pid);
    }
    cmd_vel_sub_ = controller_nh.subscribe("/cmd_vel", 10, &HeroChassisController::cmdVelCallback, this);
    odom_pub_ = controller_nh.advertise<nav_msgs::Odometry>("/odom", 50);
    controller_nh.param<bool>("use_global_mode", use_global_mode_, false);
    last_time_ = ros::Time::now();
    return true;
}
void HeroChassisController::starting(const ros::Time& time) {
    for (auto& pid : pids_) { pid.reset(); }
    last_time_ = time;
}
void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
    double r = 0.1, k = 0.5;
    double target_vel[4];
    
    // 选做7：坐标系转换
    double local_vx = target_vx_;
    double local_vy = target_vy_;
    if (use_global_mode_) {
        local_vx = target_vx_ * cos(th_) + target_vy_ * sin(th_);
        local_vy = -target_vx_ * sin(th_) + target_vy_ * cos(th_);
    }
    
    // 选做5：逆运动学
    target_vel[0] = (local_vx - local_vy - k * target_wz_) / r;
    target_vel[1] = (local_vx + local_vy + k * target_wz_) / r;
    target_vel[2] = (local_vx + local_vy - k * target_wz_) / r;
    target_vel[3] = (local_vx - local_vy + k * target_wz_) / r;

    // 选做4：PID 控制
    for (size_t i = 0; i < 4; ++i) {
        double error = target_vel[i] - joints_[i].getVelocity();
        joints_[i].setCommand(pids_[i].computeCommand(error, period));
    }

    // 选做6：正运动学与里程计
    double actual_vx = (joints_[0].getVelocity() + joints_[1].getVelocity() + joints_[2].getVelocity() + joints_[3].getVelocity()) * r / 4.0;
    double actual_vy = (-joints_[0].getVelocity() + joints_[1].getVelocity() + joints_[2].getVelocity() - joints_[3].getVelocity()) * r / 4.0;
    double actual_wz = (-joints_[0].getVelocity() + joints_[1].getVelocity() - joints_[2].getVelocity() + joints_[3].getVelocity()) * r / (4.0 * k);

    double dt = (time - last_time_).toSec();
    x_ += (actual_vx * cos(th_) - actual_vy * sin(th_)) * dt;
    y_ += (actual_vx * sin(th_) + actual_vy * cos(th_)) * dt;
    th_ += actual_wz * dt;
    last_time_ = time;

    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = time;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.translation.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);
    odom_tf.transform.rotation = odom_quat;
    tf_broadcaster_.sendTransform(odom_tf);

    nav_msgs::Odometry odom;
    odom.header.stamp = time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = actual_vx;
    odom.twist.twist.linear.y = actual_vy;
    odom.twist.twist.angular.z = actual_wz;
    odom_pub_.publish(odom);
}
void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg) {
    target_vx_ = msg.linear.x;
    target_vy_ = msg.linear.y;
    target_wz_ = msg.angular.z;
}
}
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
