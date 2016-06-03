#ifndef ROBOT_H
#define ROBOT_H

#include <queue>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include "boost/random.hpp"
#include "boost/random/normal_distribution.hpp"

typedef boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > ND_GEN;

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
    void publish_tf();
    void publish_imu();
    void publish_pose();
    void publish_control();
    bool check_reach();
    void generate_noise();
private:

    double _time, _time_old, _dt;
    double _freq;
    bool   _init_time, _init_ctrl, _init_target, _init_pose;

    double _mass;
    tf::Vector3 _force;
    tf::Vector3 _linear_acceleration;
    tf::Vector3 _linear_velocity;
    tf::Vector3 _position;

    double _inertia;
    double _target_round, _pose_round;
    tf::Vector3 _torque;
    tf::Vector3 _angular_acceleration;
    tf::Vector3 _angular_velocity;
    tf::Vector3 _orientation;
    tf::Vector3 _orientation_prev;
    tf::Quaternion _quaternion;

    tf::Vector3 _target_position;
    tf::Vector3 _target_orientation;
    tf::Vector3 _target_orientation_prev;


    double _pos_p, _pos_i, _pos_d;
    double _ori_p, _ori_i, _ori_d;

    tf::Vector3 _pos_err_i, _ori_err_i;
    tf::Vector3 _pos_err_d, _ori_err_d;
    tf::Vector3 _pos_err, _ori_err;
    tf::Vector3 _pos_err_prev, _ori_err_prev;

    double _MAX_FORCE, _MAX_TORQUE;
    double _MAX_LINEAR_VEL, _MAX_ANGULAR_VEL;

    ros::Subscriber _model_sub, _target_sub;
    ros::Publisher  _model_pub, _pose_pub, _imu_pub, _force_pub, _torque_pub;

    double _gravity;
    std::queue<sensor_msgs::Imu> _imu_queue;
    tf::Vector3 _acc_noise, _acc_sgm;
    tf::Vector3 _gyr_noise, _gyr_sgm;

    tf::Vector3 _acc_bias, _acc_bias_noise, _acc_bias_sgm;
    tf::Vector3 _gyr_bias, _gyr_bias_noise, _gyr_bias_sgm;

    boost::mt19937 _rng;

    tf::TransformBroadcaster _tf_br;

    tf::Matrix3x3 _rot_gazebo_to_dji;
    tf::Matrix3x3 _rot_dji_to_base;
    tf::Matrix3x3 _rot_gazebo_to_base;
    tf::Matrix3x3 _rot_world_to_gazebo;
    tf::Matrix3x3 _rot_world_to_base;

};
#endif // ROBOT_H
