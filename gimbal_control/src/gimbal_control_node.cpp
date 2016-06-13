#include "gimbal_control/gimbal_control.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_control_node");
    ros::NodeHandle n;

    GimbalControl gc(n);

    ros::Timer timer = n.createTimer(ros::Duration(0.01), &GimbalControl::publish_heading, &gc);
    ros::spin();

    return 0;
}
