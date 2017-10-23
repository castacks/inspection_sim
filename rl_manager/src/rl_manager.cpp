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
    ROS_INFO("START");
    
    init_variables();
    ROS_INFO("INIT VARABILES");    
    generate_random_pose();
    ROS_INFO("GEN POSE");    
    reset_sim_pose();
    ROS_INFO("SIM POSE RESET");
    _reset_pub = nh.advertise<geometry_msgs::Pose>("/dji_sim/reset", 100);
    
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = _x;
    pose_msg.position.y = _y;
    pose_msg.position.z = _z;
    _reset_pub.publish(pose_msg);

    image_transport::ImageTransport it(nh);
    _image_pub = it.advertise("range_image",1);
    _debug_image_pub = it.advertise("combined_image",1);
    
    _obstacle_pub = nh.advertise<std_msgs::Float64>("/dji_sim/closest_obstacle", 100);
    _laser_pub = nh.advertise<sensor_msgs::PointCloud2>("/dji_sim/laser_state_cloud", 100);
    _state_pub = nh.advertise<std_msgs::Float64MultiArray>("/dji_sim/laser_state", 100);
    _done_pub = nh.advertise<std_msgs::Bool>("/dji_sim/collision", 100);
    _marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    // _odom_sub = nh.subscribe("/dji_sim/odometry", 1, &RL_manager::odom_callback, this);
    
    // _target_sub = nh.subscribe("/target_dir", 1, &RL_manager::target_callback, this);
    ROS_INFO("Setup done. initing subscribers");
    
    _laser_sub_0 = nh.subscribe("/dji_sim/laser/laserscan_0", 1, &RL_manager::publish_laser_0, this);
    _laser_sub_1 = nh.subscribe("/dji_sim/laser/laserscan_1", 1, &RL_manager::publish_laser_1, this);
    _laser_sub_2 = nh.subscribe("/dji_sim/laser/laserscan_2", 1, &RL_manager::publish_laser_2, this);
    _laser_sub_3 = nh.subscribe("/dji_sim/laser/laserscan_3", 1, &RL_manager::publish_laser_3, this);
    _laser_sub_4 = nh.subscribe("/dji_sim/laser/laserscan_4", 1, &RL_manager::publish_laser_4, this);
    _laser_sub_5 = nh.subscribe("/dji_sim/laser/laserscan_5", 1, &RL_manager::publish_laser_5, this);
    
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
        if(collision){
            std_msgs::Bool done_msg;
            done_msg.data = true;
            _done_pub.publish(done_msg);
        }


    }
}

// takes in points in world cords
bool RL_manager::check_occupancy(float x, float y, float z, float radius, bool use_cov)
{
    octomap::point3d obstacle;
    octomap::point3d p(x,y,z);
    // std::cout << p << std::endl;
    float dist;
    //get obstacle in world cords
    // std::cout << p.x() << " " << p.y() << " " << p.z() << std::endl;
    _map_ptr->get_closest_obstacle(p,dist,obstacle);
    geometry_msgs::PointStamped Point_msg;
    Point_msg.point.x = obstacle.x();
    Point_msg.point.y = obstacle.y();
    Point_msg.point.z = obstacle.z();
    Point_msg.header.frame_id = "world";
    _listener.transformPoint("base_link",Point_msg,Point_msg);
    // std_msgs::Float64 float_msg;
    // float_msg.data = dist;
    // _obstacle_pub.publish(float_msg);
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
    std::uniform_real_distribution<> x_dis(_x_min, _x_max);
    std::uniform_real_distribution<> y_dis(_y_min, _y_max);
    std::uniform_real_distribution<> z_dis(_z_min, _z_max);
    std::uniform_real_distribution<> dis(0, 1);
    
    while(!found_good_pose && ros::ok())
    {
        _x = x_dis(gen);
        _y = y_dis(gen);
        _z = z_dis(gen);
        // _theta = (2*M_PI)*((double) rand() / (RAND_MAX));
        _theta = 0.0;
        // CHECK OCCUPANCY
        
        found_good_pose = !check_occupancy(_x,_y,_z,1.25*_radius,false);
        // found_good_pose = _calc_localizability_ptr->check_neighborhood_occupancy(_x,_y,_z,_radius);
    }
    std::uniform_real_distribution<> target_dis(-_target_offset, _target_offset);
    _target_x = _x + target_dis(gen)*(dis(gen));
    _target_y = _y + target_dis(gen)*(dis(gen));
    _target_z = _z + target_dis(gen)*(dis(gen));
    // double target_distance = sqrt(_target_x*_target_x + _target_y*_target_y + _target_z*_target_z);
    // _target_x = _target_x;
    // _target_y = _target_y;
    // _target_z = _target_z;
    // std::cout << _target_x << " " << _target_y << " " << _target_z << std::endl;
    
    // _target_theta = (2*M_PI)*(static_cast <double> (rand()) / static_cast <double> (RAND_MAX));
    _target_theta = 0.0;


    // _target_localizability = _calc_localizability_ptr->estimate_localizablity(_target_x,_target_y,_target_z,_target_theta);
}

void RL_manager::publish_laser_0(const sensor_msgs::LaserScan &msg)
{
    update_state(msg,0);
    _im_received_0 = true;
}

void RL_manager::publish_laser_1(const sensor_msgs::LaserScan &msg)
{
    update_state(msg,1);
    _im_received_1 = true;
}

void RL_manager::publish_laser_2(const sensor_msgs::LaserScan &msg)
{
    update_state(msg,2);
    _im_received_2 = true;
}

void RL_manager::publish_laser_3(const sensor_msgs::LaserScan &msg)
{
    update_state(msg,3);
    _im_received_3 = true;
}

void RL_manager::publish_laser_4(const sensor_msgs::LaserScan &msg)
{
    update_state(msg,4);
    _im_received_4 = true;
}

void RL_manager::publish_laser_5(const sensor_msgs::LaserScan &msg)
{
    update_state(msg,5);
    _im_received_5 = true;
}

void RL_manager::update_state(const sensor_msgs::LaserScan &msg, int laser_num)
{
    
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
            int pixel_value = int(255*(msg.ranges[index])/10.0);

            if((!std::isnan(msg.ranges[index])) && (!std::isinf(msg.ranges[index]) && (msg.ranges[index]<10.0) ))
            {
                _range_image.at<uchar>(i + laser_num*_rows,j) = pixel_value;
            }else{
                _range_image.at<uchar>(i + laser_num*_rows,j) = 255;
            }
            index--;
        }
    }

    tf::Transform target_transform;
    target_transform.setOrigin(tf::Vector3(_target_x,_target_y,_target_z));
    tf::Quaternion q2(0,0,0,1);
    target_transform.setRotation(q2);
    _br.sendTransform(tf::StampedTransform(target_transform,ros::Time::now(),"world","target"));

    if(_im_received_0 && _im_received_1 && _im_received_2 && _im_received_3 && _im_received_4 && _im_received_5)
    {
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", _range_image).toImageMsg();        
        _image_pub.publish(img_msg);
        _im_received_0 = false;
        _im_received_1 = false;
        _im_received_2 = false;
        _im_received_3 = false;
        _im_received_4 = false;
        _im_received_5 = false;
        cv::Mat diff_image = ((_prev_range_image - _range_image))+127;
        cv::hconcat(_range_image, diff_image, _combined_image);
        sensor_msgs::ImagePtr combined_img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", _combined_image).toImageMsg();                
        _debug_image_pub.publish(combined_img_msg);
        _range_image.copyTo(_prev_range_image);
    }
}

void RL_manager::init_variables()
{
    _done = false;

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
    _target_offset = 3.0;

    _rows = 10;
    _cols = 10;
    _init_counter = 0;
    _range_image = cv::Mat::zeros(6*_rows,_cols,CV_8U);
    _prev_range_image = cv::Mat::zeros(6*_rows,_cols,CV_8U);
    _combined_image = cv::Mat::zeros(6*_rows,2*_cols,CV_8U);

}
