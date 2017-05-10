#include "g2_control/g2_control.h"
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

G2Control::G2Control(ros::NodeHandle &nh) {
    _nh_ptr.reset(&nh);

    nh.param("/g2_control/spin_freq", _spin_freq, 1.0);

    _g2_pub    = nh.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 100);
    _laser_pub = nh.advertise<sensor_msgs::PointCloud2>("/dji_sim/laser/pointcloud", 100);

    _laser_sub = nh.subscribe("/dji_sim/laser/laserscan", 100, &G2Control::publish_point_cloud, this);

    _init = true;
    _tf_publishing = false;
    _g2_angle = 0.0;
}

void G2Control::publish_g2_angle(const ros::TimerEvent& event) {

    // To configure param dynamically
    _nh_ptr->getParam("/g2_control/spin_freq", _spin_freq);

    _time_now = ros::Time::now().toSec();
    _time_delta = _init ? 0.01 : _time_now - _time_old;
    _time_old = _time_now;
    _init = false;

    if(fabs(_time_delta) > 0.02 || fabs(_time_delta) < 0.005 ) {
      //   ROS_WARN("g2_control: delta_t = %0.4f", _time_delta);
        _time_delta = 0.01;
    }

    _g2_angle = M_PI*_spin_freq*_time_delta;

    gazebo_msgs::LinkState msg;
    msg.link_name = "dji::hokuyo_link";
    tf::Quaternion q;
    q.setRPY(_g2_angle, 0.0, 0.0);
    msg.pose.orientation.w = q.w();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.reference_frame = "dji::hokuyo_link";
    _g2_pub.publish(msg);


}

void G2Control::publish_point_cloud(const sensor_msgs::LaserScan &msg) {

    ROS_INFO_ONCE("Got laser scan");
    if(!_listener.waitForTransform(
        msg.header.frame_id,
        "/base_link",
        msg.header.stamp + ros::Duration().fromSec(msg.ranges.size()*msg.time_increment),
        ros::Duration(0.02))){
      //   ROS_WARN("g2_control: lose transform from %s to base_link", msg.header.frame_id.c_str());
        return;
    }

    sensor_msgs::PointCloud pc_msg;
    _projector.transformLaserScanToPointCloud("/base_link",msg,
              pc_msg,_listener);

    sensor_msgs::PointCloud2 pc2_msg;
    sensor_msgs::convertPointCloudToPointCloud2(pc_msg, pc2_msg);

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(pc2_msg,pcl_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr;
    pcl_cloud_ptr = pcl_cloud.makeShared();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_crop_x_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_crop_xy_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_crop_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    // truncating in a range
    double rangeLim = 15.0;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_cloud_ptr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits (-rangeLim, rangeLim);
    pass.filter(*pcl_cloud_crop_x_ptr);

    pass.setInputCloud(pcl_cloud_crop_x_ptr);
    pass.setFilterFieldName("y");
    pass.setFilterLimits (-rangeLim, rangeLim);
    pass.filter(*pcl_cloud_crop_xy_ptr);

    pass.setInputCloud(pcl_cloud_crop_xy_ptr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits (-rangeLim, rangeLim);
    pass.filter(*pcl_cloud_crop_xyz_ptr);

    // truncate in a bounding range
    pcl::ConditionOr<pcl::PointXYZ>::Ptr rangeCond (new pcl::ConditionOr<pcl::PointXYZ> ());
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, -0.5)));
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.5)));
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, -0.5)));
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, 0.5)));
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, -0.5)));
    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, 0.5)));

    pcl::ConditionalRemoval<pcl::PointXYZ> condRem;
    condRem.setCondition((rangeCond));
    condRem.setInputCloud(pcl_cloud_crop_xyz_ptr);
    condRem.setKeepOrganized(true);
    condRem.filter(pcl_cloud);

    sensor_msgs::PointCloud2 pc2_msg_out;
    pcl::toROSMsg(pcl_cloud, pc2_msg_out);
    pc2_msg_out.header.frame_id = "/base_link_est";
    _laser_pub.publish(pc2_msg_out);
}
