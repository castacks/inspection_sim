#include "rl_manager/rl_manager.h"
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>



RL_manager::RL_manager()
{
    ros::NodeHandle nh;

    _map_ptr = boost::shared_ptr<DistMap>(new DistMap(nh));

    // reset();
    init_variables();
    generate_random_pose();
    reset_sim_pose();
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = _x;
    pose_msg.position.x = _y;
    pose_msg.position.x = _z;
    _reset_pub.publish(pose_msg);

    //
    image_transport::ImageTransport it(nh);
    _image_pub = it.advertise("range_image",1);


    _laser_pub = nh.advertise<sensor_msgs::PointCloud2>("/dji_sim/laser_state_cloud", 100);
    _state_pub = nh.advertise<std_msgs::Float64MultiArray>("/dji_sim/laser_state", 100);
    _done_pub = nh.advertise<std_msgs::Bool>("/dji_sim/collision", 100);
    _reset_pub = nh.advertise<geometry_msgs::Pose>("/dji_sim/reset", 100);
    _marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    // _odom_sub = nh.subscribe("/dji_sim/odometry", 1, &RL_manager::odom_callback, this);
    
    // _target_sub = nh.subscribe("/target_dir", 1, &RL_manager::target_callback, this);
    _imu_sub = nh.subscribe("/dji_sim/imu", 1, &RL_manager::imu_callback, this);
    _laser_sub = nh.subscribe("/dji_sim/laser/laserscan", 1, &RL_manager::publish_state, this);

    _client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceServer reset_service = nh.advertiseService("/dji_sim/reset_service", &RL_manager::reset, this);
    // <std_srvs::Empty::Request, std_srvs::Empty::Response>
    ros::Rate r(20);
    while(ros::ok())
    {
        ros::spinOnce();
        // r.sleep();

        tf::StampedTransform transform;
        try{
            _listener.lookupTransform("gazebo_world","dji_link", ros::Time(0), transform);
        }
        catch (std::exception& e)
        {
            std::cout << e.what() << '\n';
            continue;
        }
        tf::Vector3 v = transform.getOrigin();
        // std::cout << "Current TRANSFORM: " << v.x() << " " << v.y() << " " << v.z() << " " << _x << " " << _y << " " << _z<<  std::endl;
        bool collision = check_occupancy(v.x(),-v.y(),-v.z(),_radius,false);

        if(collision || v.x() > 2*_x_max || v.x() < 2*_x_min || v.y() > 2*_y_max || v.y() < 2*_y_min){
            // std_srvs::Empty::Request request;
            // std_srvs::Empty::Response response;
            // reset(request, response);
            std_msgs::Bool done_msg;
            done_msg.data = true;
            _done_pub.publish(done_msg);
        }


    }
}

// void RL_manager::odom_callback(const nav_msgs::Odometry &msg)
// {
//     _robot_x = msg.pose.pose.position.x;
//     _robot_y = msg.pose.pose.position.y; 
//     _robot_z = msg.pose.pose.position.z; 
// }

// takes in points in world cords
bool RL_manager::check_occupancy(float x, float y, float z, float radius, bool use_cov)
{
    octomap::point3d obstacle;
    octomap::point3d p(x,y,z);
    // std::cout << p << std::endl;
    float dist;
    //get obstacle in world cords
    _map_ptr->get_closest_obstacle(p,dist,obstacle);
    geometry_msgs::PointStamped Point_msg;
    Point_msg.point.x = obstacle.x();
    Point_msg.point.y = obstacle.y();
    Point_msg.point.z = obstacle.z();
    Point_msg.header.frame_id = "world";
    _listener.transformPoint("base_link",Point_msg,Point_msg);
    // std::cout << obstacle << std::endl;
    if(use_cov){
        //use control cov to check for 3 sigma safety

    }else{
        // std::cout << dist << std::endl;
        if(dist>radius || dist == -1){
            return false;
        }else{
            return true;
        }
    }
    return false;
}

// generates random pose and publishes reset message to reset all nodes to new pose
bool RL_manager::reset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    init_variables();
    generate_random_pose();
    // std::cout <<"NEW RANDOM POSE AT: " << _x << " " << _y << " " << _z << std::endl;
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = _x;
    pose_msg.position.y = -_y;
    pose_msg.position.z = -_z;
    _reset_pub.publish(pose_msg);
    reset_sim_pose();

    // sleep
    // return finished
    return true;
}

void RL_manager::reset_sim_pose(){
    gazebo_msgs::SetModelState setmodelstate;
    gazebo_msgs::ModelState new_state = gazebo_msgs::ModelState();

    new_state.model_name = "dji";
    new_state.pose.position.x = _x;
    new_state.pose.position.y = -_y;
    new_state.pose.position.z = -_z;
    setmodelstate.request.model_state = new_state;
    _client.call(setmodelstate);
}

void RL_manager::generate_random_pose()
{
    bool found_good_pose = false;
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0, 1);
    
    while(!found_good_pose && ros::ok())
    {
        std::cout << dis(gen) << std::endl;
        _x = (_x_max-_x_min)*dis(gen) + _x_min;
        _y = (_y_max-_y_min)*dis(gen) + _y_min;
        _z = (_z_max-_z_min)*dis(gen) + _z_min;
        // _theta = (2*M_PI)*((double) rand() / (RAND_MAX));
        _theta = 0.0;
        // CHECK OCCUPANCY
        found_good_pose = !check_occupancy(_x,_y,_z,1.25*_radius,false);
        // found_good_pose = _calc_localizability_ptr->check_neighborhood_occupancy(_x,_y,_z,_radius);
    }
    _target_x = _x + _target_offset*(2*dis(gen));
    _target_y = _y + _target_offset*(2*dis(gen)-1);
    _target_z = _z + _target_offset*(2*dis(gen)-1);
    // double target_distance = sqrt(_target_x*_target_x + _target_y*_target_y + _target_z*_target_z);
    // _target_x = _target_x;
    // _target_y = _target_y;
    // _target_z = _target_z;
    // std::cout << _target_x << " " << _target_y << " " << _target_z << std::endl;
    
    // _target_theta = (2*M_PI)*(static_cast <double> (rand()) / static_cast <double> (RAND_MAX));
    _target_theta = 0.0;


    // _target_localizability = _calc_localizability_ptr->estimate_localizablity(_target_x,_target_y,_target_z,_target_theta);
}

void RL_manager::imu_callback(const sensor_msgs::Imu &msg)
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

void RL_manager::publish_state(const sensor_msgs::LaserScan &msg)
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
        ROS_WARN("rl_manager: lose transform from %s to base_link in rl_manager", msg.header.frame_id.c_str());
        return;
    }

    tf::StampedTransform transform;
    _listener.lookupTransform(msg.header.frame_id, "/base_link",  
                             ros::Time(0), transform);

    sensor_msgs::PointCloud pc_msg;
    _projector.transformLaserScanToPointCloud("/base_link",msg,pc_msg,_listener,10.0);

    geometry_msgs::PointStamped Point_msg;
    Point_msg.point.x = _target_x;
    Point_msg.point.y = _target_y;
    Point_msg.point.z = _target_z;
    Point_msg.header.frame_id = "world";
    _listener.transformPoint("dji_link",Point_msg,Point_msg);
    _target_dir[0] = Point_msg.point.x;
    _target_dir[1] = Point_msg.point.y;
    _target_dir[2] = Point_msg.point.z;
    
    int num_rays = ((msg.angle_max - msg.angle_min)/msg.angle_increment) +1;
    int index = num_rays*num_rays-1;
    // ROS_INFO(num_rays);
    // ROS_INFO(index);
    for(int i = 0; i < num_rays; i++)
    {
        for(int j = 0; j < num_rays; j++)
        {
            int pixel_value = int(255*(msg.ranges[index])/20.0);

            if((!isnan(msg.ranges[index])) && (!isinf(msg.ranges[index]) && (msg.ranges[index]<20.0) ))
            {
                _range_image.at<uchar>(i,j) = pixel_value;
            }else{
                _range_image.at<uchar>(i,j) = 255;
            }
            index--;
        }
    }

    // {
    //     index = i;
    //     if(roll_bin>16){
    //         index = num_scans-i-1;
    //     }else{
    //         index = i;
    //     }
    //     // std::cout << msg.ranges[i] << std::endl;
    //     if((!isnan(msg.ranges[index])) && (!isinf(msg.ranges[index]) && (msg.ranges[index]<10.0) )){

    //         int pixel_value = int(255*(msg.ranges[index])/10.0);
    //         // std::cout << pixel_value << " " << msg.ranges[index] << std::endl;
    //         // std::cout << _range_image.at<uchar>(0,0) << std::endl;
    //         _range_image.at<uchar>(roll_bin % 16,i/8) = pixel_value;            
    //     }else{
    //         // std::cout << "NAN OR INF" << std::endl;
    //         // std::cout << _range_image.at<uchar>(roll_bin,i) << std::endl;
    //         _range_image.at<uchar>(roll_bin % 16,i/8) = 255;            
    //     }
        
    // }
    tf::Transform target_transform;
    target_transform.setOrigin(tf::Vector3(_target_x,_target_y,_target_z));
    tf::Quaternion q2(0,0,0,1);
    target_transform.setRotation(q2);
    _br.sendTransform(tf::StampedTransform(target_transform,ros::Time::now(),"world","target"));

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", _range_image).toImageMsg();

    _image_pub.publish(img_msg);

}

void RL_manager::init_variables()
{
    _done = false;
    _N = 66;
    // 66 uniform points from meshlab
    _dirs = (Eigen::MatrixXf(_N,3) << -0.000000, -0.000000, -1.000000,
                                    -0.000000, -0.382683, -0.923880,
                                    -0.382683, 0.000000, -0.923880,
                                    0.382683, 0.000000, -0.923880,
                                    0.000000, 0.382683, -0.923880,
                                    -0.408248, -0.408248, -0.816497,
                                    0.408248, -0.408248, -0.816497,
                                    -0.408248, 0.408248, -0.816497,
                                    0.408248, 0.408248, -0.816497,
                                    -0.000000, -0.707107, -0.707107,
                                    -0.707107, 0.000000, -0.707107,
                                    0.707107, 0.000000, -0.707107,
                                    0.000000, 0.707107, -0.707107,
                                    -0.408248, -0.816497, -0.408248,
                                    0.408248, -0.816497, -0.408248,
                                    -0.816497, -0.408248, -0.408248,
                                    0.816497, -0.408248, -0.408248,
                                    -0.816497, 0.408248, -0.408248,
                                    0.816497, 0.408248, -0.408248,
                                    -0.408248, 0.816497, -0.408248,
                                    0.408248, 0.816497, -0.408248,
                                    -0.000000, -0.923880, -0.382683,
                                    -0.923880, -0.000000, -0.382683,
                                    0.923880, -0.000000, -0.382683,
                                    0.000000, 0.923880, -0.382683,
                                    -0.000000, -1.000000, 0.000000,
                                    -0.382683, -0.923880, -0.000000,
                                    0.382683, -0.923880, -0.000000,
                                    -0.707107, -0.707107, 0.000000,
                                    0.707107, -0.707107, 0.000000,
                                    -0.923880, -0.382683, -0.000000,
                                    0.923880, -0.382683, -0.000000,
                                    -1.000000, 0.000000, 0.000000,
                                    1.000000, 0.000000, 0.000000,
                                    -0.923880, 0.382683, -0.000000,
                                    0.923880, 0.382683, -0.000000,
                                    -0.707107, 0.707107, 0.000000,
                                    0.707107, 0.707107, 0.000000,
                                    -0.382683, 0.923880, -0.000000,
                                    0.382683, 0.923880, -0.000000,
                                    0.000000, 1.000000, 0.000000,
                                    0.000000, -0.923880, 0.382683,
                                    -0.923880, -0.000000, 0.382683,
                                    0.923880, -0.000000, 0.382683,
                                    -0.000000, 0.923880, 0.382683,
                                    -0.408248, -0.816497, 0.408248,
                                    0.408248, -0.816497, 0.408248,
                                    -0.816497, -0.408248, 0.408248,
                                    0.816497, -0.408248, 0.408248,
                                    -0.816497, 0.408248, 0.408248,
                                    0.816497, 0.408248, 0.408248,
                                    -0.408248, 0.816497, 0.408248,
                                    0.408248, 0.816497, 0.408248,
                                    -0.000000, -0.707107, 0.707107,
                                    -0.707107, 0.000000, 0.707107,
                                    0.707107, 0.000000, 0.707107,
                                    0.000000, 0.707107, 0.707107,
                                    -0.408248, -0.408248, 0.816497,
                                    0.408248, -0.408248, 0.816497,
                                    -0.408248, 0.408248, 0.816497,
                                    0.408248, 0.408248, 0.816497,
                                    0.000000, -0.382683, 0.923880,
                                    -0.382683, -0.000000, 0.923880,
                                    0.382683, -0.000000, 0.923880,
                                    -0.000000, 0.382683, 0.923880,
                                    0.000000, 0.000000, 1.000000).finished();

    for (int i = 0; i < _N; ++i)
    {
        _laser_state.push_back(-10.0);
    }

    // _x_min = -15;
    // _x_max = 15;

    // _y_min = -15;
    // _y_max = 15;
    _x_min = -10;
    _x_max = 10;

    _y_min = -9;
    _y_max = 9;
    
    _z_min = -12;
    _z_max = -1.25;
    
    // _robot_x = 0;
    // _robot_y = 0;
    // _robot_z = -1.5;

    _init = true;
    _tf_publishing = false;
    _prev_roll = 0;
    
    _target_state[0] = 0;
    _target_state[1] = 0;
    _target_state[2] = 0;
    _target_theta = 0;
    _target_localizability = 0;
    _radius = 1.0;
    _imu_ready = false;
    _target_offset = 3.0;

    _rows = 20;
    _cols = 20;
    _init_counter = 0;
    // range_image = sensor_msgs::Image();
    // range_image.height = _rows;
    // range_image.width = _cols;
    // range_image.step = _cols; //fix this
    _range_image = cv::Mat::zeros(_rows,_cols,CV_8U);
    std::cout << "RANGE IMAGE CREATED" <<std::endl;
    // std::cout << _range_image.at<uchar>(0,0) << std::endl;

    
}




    // sensor_msgs::PointCloud2 pc2_msg;
    // sensor_msgs::convertPointCloudToPointCloud2(pc_msg, pc2_msg);

    // pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    // pcl::fromROSMsg(pc2_msg,pcl_cloud);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr;
    // pcl_cloud_ptr = pcl_cloud.makeShared();

    // Eigen::Vector3f p; 

    // bool reset_bin = false;
    // if(roll_bin != _prev_roll){
    //     reset_bin = true;
    // }
    
    // int prev_index = 0;
    // for (int i = 0; i < pcl_cloud.size(); ++i){

    //     p(0) = pcl_cloud.points[i].x;
    //     p(1) = pcl_cloud.points[i].y;
    //     p(2) = pcl_cloud.points[i].z;

    //     double distance = p.norm();
    //     p = p/distance;
    //     Eigen::MatrixXf::Index maxIndex;
    //     float max_value = (_dirs*p).maxCoeff(&maxIndex);

    //     if(reset_bin){
    //         _laser_state[maxIndex] = 10;
    //     }
    //     if(distance < _laser_state[maxIndex] || (_laser_state[maxIndex] < 0)){
    //         _laser_state[maxIndex] = distance;
    //     }
    //     // int int_dist = floor(distance);
    //     // if(int_dist < _laser_state[maxIndex] || (_laser_state[maxIndex] < 0)){
    //     //     _laser_state[maxIndex] = int_dist;
    //     // }
    //     prev_index = int(maxIndex);
    // }

    // // if(roll_bin == _prev_roll){
    // //     return;
    // // }
    // // _prev_roll = roll_bin;


//   // std::cout << std::endl;
//     std_msgs::Float64MultiArray state_msg;
//     state_msg.data.clear();
    
//     pcl::PointCloud<pcl::PointXYZ> state_cloud;
//     pcl::PointXYZ state_point;
        
//     for (int i = 0; i < _N; ++i)
//     {
//         // std::cout << _laser_state[i] << ", ";
//         state_msg.data.push_back(_laser_state[i]);
//         if(_laser_state[i] < 0){
//             continue;
//         }
//         state_point.x = _dirs(i,0)*_laser_state[i];
//         state_point.y = _dirs(i,1)*_laser_state[i];
//         state_point.z = _dirs(i,2)*_laser_state[i];
//         state_cloud.push_back(state_point);
//     }

//     tf::Transform target_transform;
//     target_transform.setOrigin(tf::Vector3(_target_x,_target_y,_target_z));
//     tf::Quaternion q2(0,0,0,1);
//     target_transform.setRotation(q2);
//     _br.sendTransform(tf::StampedTransform(target_transform,ros::Time::now(),"world","target"));

//     // state_msg.data.push_back(_angular_velocity[0]);
//     // state_msg.data.push_back(_angular_velocity[1]);
//     // state_msg.data.push_back(_angular_velocity[2]);

//     // state_msg.data.push_back(_linear_acceleration[0]);
//     // state_msg.data.push_back(_linear_acceleration[1]);
//     // state_msg.data.push_back(_linear_acceleration[2]);

//     state_msg.data.push_back(_target_dir[0]);
//     state_msg.data.push_back(_target_dir[1]);
//     state_msg.data.push_back(_target_dir[2]);
//     // state_msg.data.push_back(_target_localizability);
    
//     // state_msg.data.push_back(_target_theta);

//     // state_msg.data.push_back(_current_pose[0]);
//     // state_msg.data.push_back(_current_pose[1]);
//     // state_msg.data.push_back(_current_pose[2]);
//     // state_msg.data.push_back(_current_orientation[2]);


//     // std::cout << std::endl;
//     _state_pub.publish(state_msg);
    
//     sensor_msgs::PointCloud2 pc2_msg_out;
//     pcl::toROSMsg(state_cloud, pc2_msg_out);
//     pc2_msg_out.header.frame_id = "/base_link";
//     _laser_pub.publish(pc2_msg_out);
//     // ROS_INFO("PUBLISHING STATE");