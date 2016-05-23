#ifndef G2_CONTROL_H
#define G2_CONTROL_H

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <gazebo_msgs/LinkState.h>

class G2Control {
public:
    G2Control(ros::NodeHandle &nh);
    ~G2Control() {}

    void publish_point_cloud(const sensor_msgs::LaserScan &msg);
    void publish_g2_angle(const ros::TimerEvent& event);
private:
    ros::NodeHandlePtr _nh_ptr;
    double _spin_freq;
    double _g2_angle;
    bool   _init, _tf_publishing;
    double _time_now, _time_old, _time_delta;

    ros::Publisher  _g2_pub, _laser_pub;
    ros::Subscriber _laser_sub;

    tf::TransformBroadcaster _br;
    laser_geometry::LaserProjection _projector;
    tf::TransformListener _listener;
};

#endif // G2_CONTROL_H
