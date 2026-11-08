#pragma once
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

namespace hero_chassis_controller {
class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
    void starting(const ros::Time& time) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
private:
    void cmdVelCallback(const geometry_msgs::Twist& msg);
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<control_toolbox::Pid> pids_;
    ros::Subscriber cmd_vel_sub_;
    double target_vx_ = 0.0;
    double target_vy_ = 0.0;
    double target_wz_ = 0.0;
    double x_ = 0.0;
    double y_ = 0.0;
    double th_ = 0.0;
    ros::Time last_time_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    bool use_global_mode_ = false; // 选做7：全局模式开关
};
}
