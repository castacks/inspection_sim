#include "g2_control/g2_control.h"
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

G2Control::G2Control() {

    ros::NodeHandle nh;
    // _nh_ptr.reset(&nh);

    nh.param("/g2_control/spin_freq", _spin_freq, 1.0);

    _done = false;
    // double _range_image[4][8] = {{-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0},
    //                              {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0},
    //                              {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0},
    //                              {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0}};

    // double _range_image_final[4][8] = {{-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0},
    //                              {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0},
    //                              {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0},
    //                              {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0}};

    // std::vector<double> _laser_state;

    for (int i = 0; i < 26; ++i)
    {
        _laser_state.push_back(-10.0);
    }


    _dirs = (Eigen::MatrixXf(26,3) << 1, 1, 1,
                                      1, 1, 0,
                                      1, 1,-1,
                                      1, 0, 1,
                                      1, 0, 0,
                                      1, 0,-1,
                                      1,-1, 1,
                                      1,-1, 0,
                                      1,-1,-1,
                                      0, 1, 1,
                                      0, 1, 0,
                                      0, 1,-1,
                                      0, 0, 1,
                                      0, 0,-1,
                                      0,-1, 1,
                                      0,-1, 0,
                                      0,-1,-1,
                                     -1, 1, 1,
                                     -1, 1, 0,
                                     -1, 1,-1,
                                     -1, 0, 1,
                                     -1, 0, 0,
                                     -1, 0,-1,
                                     -1,-1, 1,
                                     -1,-1, 0,
                                     -1,-1,-1).finished();

    // std::cout << _dirs << std::endl;
    for (int i = 0; i < 26; ++i)
    {
        // std::cout << _dirs.row(i) << std::endl;
        _dirs.row(i) = _dirs.row(i)/(_dirs.row(i).norm());
    }



    _g2_pub    = nh.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 100);
    _laser_pub = nh.advertise<sensor_msgs::PointCloud2>("/dji_sim/laser/pointcloud", 100);
    _state_pub = nh.advertise<std_msgs::Float64MultiArray>("/dji_sim/laser_state", 100);
    _done_pub = nh.advertise<std_msgs::Bool>("/dji_sim/is_done", 100);
    _imu_sub = nh.subscribe("/dji_sim/imu", 1, &G2Control::imu_callback, this);
    // _target_sub = nh.subscribe("/target_dir", 1, &G2Control::target_callback, this);

    _init = true;
    _tf_publishing = false;
    _g2_angle = 0.0;
    _prev_roll = 0;
    
    _target_dir[0] = 0;
    _target_dir[1] = 0;
    _target_dir[2] = 0;
    _imu_ready = false;

    _laser_sub = nh.subscribe("/dji_sim/laser/laserscan", 1, &G2Control::publish_state, this);

    // ros::Timer timer = nh.createTimer(ros::Duration(0.01), &G2Control::publish_g2_angle, this);
    // ros::spin();
}

void G2Control::publish_g2_angle(const ros::TimerEvent& event)
{
    // To configure param dynamically
    nh.getParam("/g2_control/spin_freq", _spin_freq);

    _time_now = ros::Time::now().toSec();
    _time_delta = _init ? 0.01 : _time_now - _time_old;
    _time_old = _time_now;
    _init = false;

    if(fabs(_time_delta) > 0.02 || fabs(_time_delta) < 0.005 ) {
        // ROS_WARN("g2_control: delta_t = %0.4f", _time_delta);
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

// void G2Control::gazebo_contact_callback(ConstContactsPtr &contact)
// {
//     if(contact->contact_size()>0){
//         _done = true;
//         std_msgs::Bool done_msg;
//         done_msg.data = _done;
//         _done_pub.publish(done_msg);

//         for (int i = 0; i < 26; ++i)
//         {
//             _laser_state.push_back(-10.0);
//         }
//         _init = true;
//         _tf_publishing = false;
//         _g2_angle = 0.0;
//         _prev_roll = 0;
        
//         _target_dir[0] = 0;
//         _target_dir[1] = 0;
//         _target_dir[2] = 0;
//         _imu_ready = false;
//     }
//     _done = false;

// }

void G2Control::imu_callback(const sensor_msgs::Imu &msg)
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.orientation, q);
    _orientation_matrix = tf::Matrix3x3(q);
    
    _angular_velocity[0] = msg.angular_velocity.x;
    _angular_velocity[1] = msg.angular_velocity.y;
    _angular_velocity[2] = msg.angular_velocity.z;
    
    _linear_acceleration[0] = msg.linear_acceleration.x;
    _linear_acceleration[1] = msg.linear_acceleration.y;
    _linear_acceleration[2] = msg.linear_acceleration.z;
    _imu_ready = true;
}

// // void G2Control::target_callback(const geometry_msgs::Vector3 &msg){

// // }

void G2Control::publish_state(const sensor_msgs::LaserScan &msg)
{
    // ROS_INFO("INSIDE PUBLISH STATE");

    if(!_imu_ready){
        ROS_INFO("WAITING FOR IMU DATA");
        return;
    }
    
    ROS_INFO_ONCE("Got laser scan");
    if(!_listener.waitForTransform(
        msg.header.frame_id,
        "/base_link",
        msg.header.stamp + ros::Duration().fromSec(msg.ranges.size()*msg.time_increment),
        ros::Duration(0.03))){
        ROS_WARN("g2_control: lose transform from %s to base_link", msg.header.frame_id.c_str());
        return;
    }

    tf::StampedTransform transform;
    _listener.lookupTransform(msg.header.frame_id, "/base_link",  
                             ros::Time(0), transform);
    tf::Quaternion q = transform.getRotation();
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    int roll_bin = ceil(8*((roll + M_PI)/(2*M_PI)-0.0625));
    roll_bin = roll_bin % 4;
    // std::cout << roll << " "<< roll_bin << " " << _prev_roll << std::endl;

    sensor_msgs::PointCloud pc_msg;
    _projector.transformLaserScanToPointCloud("/base_link",msg,pc_msg,_listener,30.0);

    sensor_msgs::PointCloud2 pc2_msg;
    sensor_msgs::convertPointCloudToPointCloud2(pc_msg, pc2_msg);

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(pc2_msg,pcl_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr;
    pcl_cloud_ptr = pcl_cloud.makeShared();

    Eigen::Vector3f p; 

                       
    int prev_index = 0;
    for (int i = 0; i < pcl_cloud.size(); ++i){

        p(0) = pcl_cloud.points[i].x;
        p(1) = pcl_cloud.points[i].y;
        p(2) = pcl_cloud.points[i].z;

        double distance = p.norm();
        p = p/distance;
        Eigen::MatrixXf::Index maxIndex;
        float max_value = (_dirs*p).maxCoeff(&maxIndex);

        if((prev_index!=int(maxIndex))){
            _laser_state[maxIndex] = -10.0;
        }

        if(distance < _laser_state[maxIndex] || (_laser_state[maxIndex] < 0)){
            _laser_state[maxIndex] = distance;
        }
        prev_index = int(maxIndex);
    }

    if(roll_bin == _prev_roll){
        return;
    }
    _prev_roll = roll_bin;

    tf::Vector3 world_dir;
    world_dir[0] = 1;
    world_dir[1] = 0;
    world_dir[2] = 0;
    _target_dir = _orientation_matrix*world_dir;

    // std::cout << std::endl;
    std_msgs::Float64MultiArray state_msg;
    state_msg.data.clear();
    
    pcl::PointCloud<pcl::PointXYZ> state_cloud;
    pcl::PointXYZ state_point;
        
    for (int i = 0; i < 26; ++i)
    {
        // std::cout << _laser_state[i] << ", ";
        state_msg.data.push_back(_laser_state[i]);
        
        state_point.x = _dirs(i,0)*_laser_state[i];
        state_point.y = _dirs(i,1)*_laser_state[i];
        state_point.z = _dirs(i,2)*_laser_state[i];
        state_cloud.push_back(state_point);
    }

    state_msg.data.push_back(_angular_velocity[0]);
    state_msg.data.push_back(_angular_velocity[1]);
    state_msg.data.push_back(_angular_velocity[2]);

    state_msg.data.push_back(_linear_acceleration[0]);
    state_msg.data.push_back(_linear_acceleration[1]);
    state_msg.data.push_back(_linear_acceleration[2]);

    state_msg.data.push_back(_target_dir[0]);
    state_msg.data.push_back(_target_dir[1]);
    state_msg.data.push_back(_target_dir[2]);

    // std::cout << std::endl;
    _state_pub.publish(state_msg);
    
    sensor_msgs::PointCloud2 pc2_msg_out;
    pcl::toROSMsg(state_cloud, pc2_msg_out);
    pc2_msg_out.header.frame_id = "/base_link";
    _laser_pub.publish(pc2_msg_out);
    // ROS_INFO("PUBLISHING STATE");
    

}


// // void G2Control::publish_range_image(const sensor_msgs::LaserScan &msg){
// //     tf::StampedTransform transform;
    
// //     ROS_INFO_ONCE("Got laser scan");
// //     if(!_listener.waitForTransform(
// //         msg.header.frame_id,
// //         "/base_link",
// //         msg.header.stamp + ros::Duration().fromSec(msg.ranges.size()*msg.time_increment),
// //         ros::Duration(0.04))){
// //         ROS_WARN("g2_control: lose transform from %s to base_link", msg.header.frame_id.c_str());
// //         return;
// //     }
// //     _listener.lookupTransform(msg.header.frame_id, "/base_link",  
// //                              ros::Time(0), transform);
    
// //     tf::Quaternion q = transform.getRotation();
// //     tf::Matrix3x3 m(q);
// //     double roll,pitch,yaw;
// //     m.getRPY(roll,pitch,yaw);
// //     // std::cout << roll << " " << _g2_angle << std::endl;
// //     int num_scans = ((msg.angle_max - msg.angle_min)/msg.angle_increment) +1 ;

// //     // std::cout << ceil(8*((roll + M_PI)/(2*M_PI)-0.0625)) << std::endl;
// //     int roll_bin = ceil(8*((roll + M_PI)/(2*M_PI)-0.0625));
// //     roll_bin = roll_bin % 8;
// //     // roll_bin = roll_bin % 4;

// //     if(_prev_roll != (roll_bin % 4)){
// //         for (int i = 0; i < 8; ++i)
// //         {
// //             _range_image_final[_prev_roll][i] = _range_image[_prev_roll][i];
// //             _range_image[roll_bin % 4][i] = -1.0;
// //         }
// //         std::cout << _prev_roll % 4 << " " << roll << std::endl;
// //         for (int i = 0; i < 4; ++i)
// //             {
// //             for (int j = 0; j < 8; ++j)
// //             {
// //                 std::cout << _range_image_final[i][j] << ", ";
// //             }
// //             std::cout << " " <<std::endl;
// //         }
// //         std::cout << " " <<std::endl;

// //     }
// //     _prev_roll = roll_bin % 4;

// //     int theta_bin;
// //     // std::cout <<"1. " << _range_image << std::endl;


// //     // }
// //     // std::cout <<"3. " << _range_image << std::endl;
// //     double theta;
// //     for (int i = 0; i < num_scans; ++i)
// //     {
// //         if(roll_bin < 4){
// //             theta = msg.angle_min + msg.angle_increment*i;
// //             theta_bin = ceil(8*((theta + M_PI)/(2*M_PI)-0.0625));
        
// //         }else{
// //             theta = msg.angle_min + msg.angle_increment*i;
// //             theta_bin = ceil(8*((-theta + M_PI)/(2*M_PI)-0.0625));
        
// //         }
// //         // if(roll_bin < 3){
          
// //         // }else{
// //         //     theta_bin = ceil(8*((msg.angle_min + msg.angle_increment*i + M_PI)/(2*M_PI)-0.0625));
// //         // }
// //         if(theta_bin == 4 || theta_bin == 0){
// //             if(msg.ranges[i] < _range_image[0][theta_bin] || _range_image[0][theta_bin]<0.01){
// //                 _range_image[0][theta_bin] = msg.ranges[i];
// //             }    
// //         }else{
// //             if(msg.ranges[i] < _range_image[roll_bin % 4][theta_bin] || _range_image[roll_bin % 4][theta_bin]<0.01){
// //                 _range_image[roll_bin % 4][theta_bin] = msg.ranges[i];
// //             }    
// //         }
// //     }


//     // if(!_listener.waitForTransform(
//     //     msg.header.frame_id,
//     //     "/base_link",
//     //     msg.header.stamp + ros::Duration().fromSec(msg.ranges.size()*msg.time_increment),
//     //     ros::Duration(0.02))){
//     //     ROS_WARN("g2_control: lose transform from %s to base_link", msg.header.frame_id.c_str());
//     //     return;
//     // }

// // }

// void G2Control::publish_point_cloud(const sensor_msgs::LaserScan &msg) {

//     ROS_INFO_ONCE("Got laser scan");
//     if(!_listener.waitForTransform(
//         msg.header.frame_id,
//         "/base_link",
//         msg.header.stamp + ros::Duration().fromSec(msg.ranges.size()*msg.time_increment),
//         ros::Duration(0.02))){
//         ROS_WARN("g2_control: lose transform from %s to base_link", msg.header.frame_id.c_str());
//         return;
//     }

//     sensor_msgs::PointCloud pc_msg;
//     _projector.transformLaserScanToPointCloud("/base_link",msg,
//               pc_msg,_listener);

//     sensor_msgs::PointCloud2 pc2_msg;
//     sensor_msgs::convertPointCloudToPointCloud2(pc_msg, pc2_msg);

//     pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
//     pcl::fromROSMsg(pc2_msg,pcl_cloud);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr;
//     pcl_cloud_ptr = pcl_cloud.makeShared();
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_crop_x_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_crop_xy_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_crop_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>);

//     // truncating in a range
//     double rangeLim = 15.0;
//     pcl::PassThrough<pcl::PointXYZ> pass;
//     pass.setInputCloud(pcl_cloud_ptr);
//     pass.setFilterFieldName("x");
//     pass.setFilterLimits (-rangeLim, rangeLim);
//     pass.filter(*pcl_cloud_crop_x_ptr);

//     pass.setInputCloud(pcl_cloud_crop_x_ptr);
//     pass.setFilterFieldName("y");
//     pass.setFilterLimits (-rangeLim, rangeLim);
//     pass.filter(*pcl_cloud_crop_xy_ptr);

//     pass.setInputCloud(pcl_cloud_crop_xy_ptr);
//     pass.setFilterFieldName("z");
//     pass.setFilterLimits (-rangeLim, rangeLim);
//     pass.filter(*pcl_cloud_crop_xyz_ptr);

//     // truncate in a bounding range
//     pcl::ConditionOr<pcl::PointXYZ>::Ptr rangeCond (new pcl::ConditionOr<pcl::PointXYZ> ());
//     rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//           pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, -0.5)));
//     rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//           pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.5)));
//     rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//           pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, -0.5)));
//     rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//           pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, 0.5)));
//     rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//           pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, -0.5)));
//     rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//           pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, 0.5)));

//     pcl::ConditionalRemoval<pcl::PointXYZ> condRem;
//     condRem.setCondition((rangeCond));
//     condRem.setInputCloud(pcl_cloud_crop_xyz_ptr);
//     condRem.setKeepOrganized(true);
//     condRem.filter(pcl_cloud);

//     sensor_msgs::PointCloud2 pc2_msg_out;
//     pcl::toROSMsg(pcl_cloud, pc2_msg_out);
//     _laser_pub.publish(pc2_msg_out);
// }
