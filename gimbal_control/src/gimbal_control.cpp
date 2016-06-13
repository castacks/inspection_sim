#include "gimbal_control/gimbal_control.h"

template <typename T> double sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

GimbalControl::GimbalControl(ros::NodeHandle &nh)
{
    nh.param("gimbal_velocity", _angular_vel, 90.0);

    _target_sub = nh.subscribe("/dji_sim/gimbal_target", 100, &GimbalControl::target_callback, this);
    _gimbal_pub = nh.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 100);

    _gimbal_orientation.setZero();
    _gimbal_increments.setZero();
}

void GimbalControl::target_callback(const geometry_msgs::Point &msg)
{
    // To ensure gimbal frame is defined as z down, x forward.
    _target_orientation[0] = msg.x;
    _target_orientation[1] = -msg.y;
    _target_orientation[2] = -msg.z;

    // Check if roll angle is zero
    if(_target_orientation[0] != 0.0) {
        _target_orientation[0] = 0.0;
        ROS_WARN("gimbal_control: target roll angle is not zero, forcing to be zero.");
    }

    // Check if target angle exceeds the limits
    for(int i=0; i<3; i++) {
        if (fabs(_target_orientation[i]) > M_PI/2.0) {
            ROS_WARN("gimbal_control: gimbal limit reached target_orientation[%d] = %0.4f", i, _target_orientation[i]);
            _target_orientation[i] = M_PI/2.0 * sgn(_target_orientation[i]);
        }
    }
}

void GimbalControl::compute_increment()
{
    // step = angular velocity / frequency
    double step = _angular_vel / 180.0 * M_PI / 100.0;

    _gimbal_increments = _target_orientation - _gimbal_orientation;

    for(int i=0; i<3; i++) {
        if(fabs(_gimbal_increments[i]) >= step) {
            _gimbal_increments[i] = sgn(_gimbal_increments[i]) * step;
        }
    }

    _gimbal_orientation = _gimbal_orientation + _gimbal_increments;
}

void GimbalControl::publish_heading(const ros::TimerEvent& event)
{
    compute_increment();

    gazebo_msgs::LinkState msg;
    msg.link_name = "dji::camera_link";
    tf::Quaternion q;
    q.setRPY(_gimbal_orientation[0], _gimbal_orientation[1], _gimbal_orientation[2]);
    msg.pose.orientation.w = q.w();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();

    // Always keep this identical with defined camera link in dji.xacro file
    msg.pose.position.x = 0.18;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.2;
    msg.reference_frame = "dji::dji_link";

    _gimbal_pub.publish(msg);


}
