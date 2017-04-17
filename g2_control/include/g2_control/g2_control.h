#ifndef G2_CONTROL_H
#define G2_CONTROL_H

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include "sensor_msgs/JointState.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/LinkState.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

class G2Control {
public:
    G2Control();
    ~G2Control() {}

    void publish_point_cloud(const sensor_msgs::LaserScan &msg);
    void publish_g2_angle(const ros::TimerEvent& event);
    void publish_state(const sensor_msgs::LaserScan &msg);
    void imu_callback(const sensor_msgs::Imu &msg);
    void target_callback(const geometry_msgs::Vector3 &msg);
    void gazebo_contact_callback(ConstContactsPtr &msg);
    ros::NodeHandle nh;

// private:
    // ros::NodeHandlePtr _nh_ptr;
    double _spin_freq;
    double _g2_angle;
    bool   _init, _tf_publishing, _imu_ready, _done;
    double _time_now, _time_old, _time_delta;
    
    // double _range_image[4][8];
    // double _range_image_final[4][8];
    std::vector<double> _laser_state;
    int _prev_roll;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::MatrixXf _dirs;

    ros::Publisher  _g2_pub, _laser_pub, _state_pub, _done_pub;
    ros::Subscriber _laser_sub, _imu_sub, _target_sub;

    tf::TransformBroadcaster _br;
    laser_geometry::LaserProjection _projector;
    tf::TransformListener _listener;
    tf::Vector3 _linear_acceleration;
    tf::Vector3 _angular_velocity;
    tf::Vector3 _target_dir;
    tf::Matrix3x3 _orientation_matrix;
};

#endif // G2_CONTROL_H
