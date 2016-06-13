#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/LinkStates.h>

class GimbalControl {
public:
    GimbalControl(ros::NodeHandle &nh);
    ~GimbalControl() {}

    void target_callback(const geometry_msgs::Point &msg);
    void compute_increment();
    void publish_heading(const ros::TimerEvent &event);

private:
    tf::Vector3    _gimbal_orientation;
    tf::Vector3    _gimbal_increments;
    tf::Vector3    _target_orientation;

    ros::Subscriber _target_sub;
    ros::Publisher  _gimbal_pub;

    double _angular_vel;
};
#endif // GIMBAL_CONTROL_H
