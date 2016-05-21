#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>

#include <gazebo_msgs/ModelStates.h>
class RobotControl {

public:
    RobotControl(ros::NodeHandle &nh);
    ~RobotControl() {}

    void target_callback(const geometry_msgs::Pose::Ptr msg);
    void pose_callback(const gazebo_msgs::ModelStates::Ptr &msg);
    void update_time();
    void update_control();
    void update_state();
    void publish_state();
    bool check_reach();
private:

    double _time, _time_old, _dt;
    double _freq;
    bool   _init_time, _init_ctrl;

    double _mass;
    tf::Vector3 _force;
    tf::Vector3 _linear_acceleration;
    tf::Vector3 _linear_velocity;
    tf::Vector3 _position;

    double _inertia;
    tf::Vector3 _torque;
    tf::Vector3 _angular_acceleration;
    tf::Vector3 _angular_velocity;
    tf::Vector3 _orientation;
    tf::Vector3 _target_position;
    tf::Vector3 _target_orientation;


    double _pos_p, _pos_i, _pos_d;
    double _ori_p, _ori_i, _ori_d;

    tf::Vector3 _pos_err_i, _ori_err_i;
    tf::Vector3 _pos_err_d, _ori_err_d;
    tf::Vector3 _pos_err, _ori_err;
    tf::Vector3 _pos_err_prev, _ori_err_prev;

    double _MAX_FORCE, _MAX_TORQUE;

    ros::Subscriber _pose_sub, _target_sub;
    ros::Publisher  _model_pub;

};
#endif // ROBOT_H
