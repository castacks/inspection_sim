#include "robot_control/robot.h"

RobotControl* robot_ptr;


void reset(const std_msgs::Bool &msg)
{
	if(msg.data){
		std::cout << "RESETTING!!!!!!!!" <<std::endl;
		delete(robot_ptr);
		robot_ptr = new RobotControl();	
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_control_node");
    robot_ptr = new RobotControl();
    ros::Subscriber sub = robot_ptr->nh.subscribe("/dji_sim/is_done", 100, reset);
    ros::spin();
    return 0;
}
