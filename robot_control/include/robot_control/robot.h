
#ifndef ROBOT_H
#define ROBOT_H

#include <queue>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
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
    void target_accel_callback(const geometry_msgs::Twist::Ptr msg);
    void pose_callback(const gazebo_msgs::ModelStates::Ptr &msg);
    void update_time();
    void update_control();
    void update_state();
    void publish_state();
    void publish_tf();
    void publish_imu();
    void publish_bias();
    void publish_pose();
    void publish_odom();
    void publish_control();
    bool check_reach();
    void generate_noise();
    void reset(const geometry_msgs::Pose &msg);
    void update_control_variance();
    void localization_callback(const nav_msgs::Odometry &msg);
    void update_accel_control();

private:
    ros::NodeHandlePtr _nh_ptr;
    double _time, _time_old, _dt;
    double _freq;
    bool   _init_time, _init_ctrl, _init_target, _init_pose;

    double _mass;
    tf::Vector3 _force;
    tf::Vector3 _linear_acceleration;
    tf::Vector3 _linear_velocity;
    tf::Vector3 _position;

    tf::Vector3 _est_position;
    tf::Vector3 _est_orientation;


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

    ros::Subscriber _model_sub, _target_sub, _reset_sub, _state_estumate_sub, _target_accel_sub;
    ros::Publisher  _model_pub, _force_pub, _torque_pub, _err_pub, _cov_pub;
    ros::Publisher  _odom_pub, _pose_pub, _imu_pub, _bias_pub;

    double _gravity;
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

    tf::Quaternion _imu_quaternion;

    std::vector<double> _pos_err_d_vec_x, _pos_err_d_vec_y, _pos_err_d_vec_z;
    std::vector<double> _ori_err_d_vec_x, _ori_err_d_vec_y, _ori_err_d_vec_z;
    int _err_count;



    // covariance stuff
    double _var_err_x, _var_err_y, _var_err_z;
    double _mean_err_x, _mean_err_y, _mean_err_z;
    int _err_q_counter, _err_q_length;
    std::vector<double> _delta_x_errs, _delta_x_errs_post;
    std::vector<double> _delta_y_errs, _delta_y_errs_post;
    std::vector<double> _delta_z_errs, _delta_z_errs_post;

    // Accel Stuff
    tf::Vector3 _target_accel;
    tf::Vector3 _target_angular_velocity;

    double _accel_p, _accel_i, _accel_d;
    double _ang_vel_p, _ang_vel_i, _ang_vel_d;

    tf::Vector3 _accel_err_i, _ang_vel_err_i;
    tf::Vector3 _accel_err_d, _ang_vel_err_d;
    tf::Vector3 _accel_err, _ang_vel_err;
    tf::Vector3 _accel_err_prev, _ang_vel_err_prev;

    std::vector<double> _accel_err_d_vec_x, _accel_err_d_vec_y, _accel_err_d_vec_z;
    std::vector<double> _ang_vel_err_d_vec_x, _ang_vel_err_d_vec_y, _ang_vel_err_d_vec_z;

    double _vel_p, _vel_d;
};
#endif // ROBOT_H
