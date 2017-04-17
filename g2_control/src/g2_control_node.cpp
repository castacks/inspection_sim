#include "g2_control/g2_control.h"

G2Control* g2_control_ptr;

void reset(ConstContactsPtr &contact)
{	
	// std::cout << "Checking Constacts: " << contact->contact_size() << std::endl;
	if(contact->contact_size()>0){
		std::cout << "ROBOT HAS CRASHED!!!!!!" << std::endl;
		std_msgs::Bool done_msg;
		done_msg.data = true;
		g2_control_ptr->_done_pub.publish(done_msg);
		// std::cout << "RESETTING!!!!!!!!" <<std::endl;
		// g2_control_ptr = new G2Control();	
	}
	// else{
	// 	std_msgs::Bool done_msg;
	// 	done_msg.data = false;
	// 	g2_control_ptr->_done_pub.publish(done_msg);
	// }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "g2_control_node");
    g2_control_ptr = new G2Control();

    
    // Load gazebo
    gazebo::client::setup(0,0);
    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/physics/contacts", reset);
    
    ros::Timer timer = g2_control_ptr->nh.createTimer(ros::Duration(0.01), &G2Control::publish_g2_angle, g2_control_ptr);
    ros::spin();

 //    while (true)
 //    	gazebo::common::Time::MSleep(10);

	// // Make sure to shut everything down.
	gazebo::client::shutdown();

    // ros::spin();

    return 0;
}
