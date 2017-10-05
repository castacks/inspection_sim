#ifndef RL_MANAGER_H
#define RL_MANAGER_H

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/LinkState.h>
#include <nav_msgs/Odometry.h>
#include <laser_geometry/laser_geometry.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

#include <lidar_eskf/map.h>

#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class RL_manager
{

public:
	RL_manager();
	~RL_manager();
	
	void publish_state(const sensor_msgs::LaserScan &msg);
    void imu_callback(const sensor_msgs::Imu &msg);
    void target_callback(const geometry_msgs::Vector3 &msg);
    void odom_callback(const nav_msgs::Odometry &msg);
    void generate_random_pose();
    bool reset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    void init_variables();
    void reset_sim_pose();
    bool check_occupancy(float x, float y, float z, float radius, bool use_cov);

    void publish_laser_0(const sensor_msgs::LaserScan &msg);
    void publish_laser_1(const sensor_msgs::LaserScan &msg);
    void publish_laser_2(const sensor_msgs::LaserScan &msg);
    void publish_laser_3(const sensor_msgs::LaserScan &msg);
    void publish_laser_4(const sensor_msgs::LaserScan &msg);
    void publish_laser_5(const sensor_msgs::LaserScan &msg);

    ros::NodeHandle nh;

    bool   _init, _tf_publishing, _imu_ready, _done;
    double _time_now, _time_old, _time_delta;
    
    std::vector<double> _laser_state;
    int _prev_roll, _N;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::MatrixXf _dirs;

    ros::Publisher  _g2_pub, _laser_pub, _state_pub, _done_pub, _reset_pub, _marker_pub;
    ros::Subscriber _laser_sub, _imu_sub, _target_sub, _odom_sub;
    ros::Subscriber _laser_sub_0;
    ros::Subscriber _laser_sub_1;
    ros::Subscriber _laser_sub_2;
    ros::Subscriber _laser_sub_3;
    ros::Subscriber _laser_sub_4;
    ros::Subscriber _laser_sub_5;
    
    image_transport::Publisher _image_pub_0;
    image_transport::Publisher _image_pub_1;
    image_transport::Publisher _image_pub_2;
    image_transport::Publisher _image_pub_3;
    image_transport::Publisher _image_pub_4;
    image_transport::Publisher _image_pub_5;

    ros::ServiceClient _client;


    laser_geometry::LaserProjection _projector;

    tf::TransformBroadcaster _br;
    tf::TransformListener _listener;
    tf::Vector3 _linear_acceleration;
    tf::Vector3 _angular_velocity;
    tf::Vector3 _target_state, _current_pose, _current_orientation, _target_dir;
    tf::Matrix3x3 _orientation_matrix;

	double _x_max;
	double _y_max;
	double _z_max;
	double _x_min;
	double _y_min;
	double _z_min;
	double _x, _y, _z, _theta, _radius;
	double _target_x, _target_y, _target_z, _target_theta, _target_offset, _target_localizability;	

	boost::shared_ptr<DistMap> _map_ptr;
    int _rows, _cols;
    image_transport::Publisher _image_pub;
    cv::Mat _range_image;
    
    double _robot_x, _robot_y, _robot_z;
    int _init_counter;
};



#endif // RL_MANAGER_H
