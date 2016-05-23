#include "robot_control/robot.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_control_node");
    ros::NodeHandle n;

    RobotControl robot(n);
//    ros::Timer timer = n.createTimer(ros::Duration(0.05), &RobotControl::publish_tf, &robot);
    ros::spin();

    return 0;
}
