#include "g2_control/g2_control.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "g2_control_node");
    ros::NodeHandle n;

    G2Control g2(n);
    ros::Timer timer = n.createTimer(ros::Duration(0.01), &G2Control::publish_g2_angle, &g2);
    ros::spin();

    return 0;
}
