#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

class JSControl {
public:
    JSControl(ros::NodeHandle &nh);
    ~JSControl() {}

    void js_callback(const sensor_msgs::Joy &msg);
    void target_callback(const geometry_msgs::Pose &msg);
    void publish_target(const ros::TimerEvent& event);
    void update_time();

private:
    tf::Vector3 _target_position;
    tf::Vector3 _target_orientation;
    tf::Quaternion _target_quaternion;
    tf::Matrix3x3 _target_rotation;

    double _x_rate, _y_rate, _z_rate, _yaw_rate;
    double _MAX_X_RATE, _MAX_Y_RATE, _MAX_Z_RATE, _MAX_YAW_RATE;

    double _time, _time_old, _dt;
    double _freq;
    bool _init_time;

    ros::Publisher _target_pub;
    ros::Subscriber _js_sub, _target_sub;
};

JSControl::JSControl(ros::NodeHandle &nh)
{
    nh.param("max_x_rate",      _MAX_X_RATE,    0.3);
    nh.param("max_y_rate",      _MAX_Y_RATE,    0.3);
    nh.param("max_z_rate",      _MAX_Z_RATE,    0.3);
    nh.param("max_yaw_rate",    _MAX_YAW_RATE,  0.1);

    _target_position.setZero();
    _target_orientation.setZero();
    _target_rotation.setIdentity();

    _x_rate = 0.0;
    _y_rate = 0.0;
    _z_rate = 0.0;
    _yaw_rate = 0.0;

    _target_pub = nh.advertise<geometry_msgs::Pose>("/dji_sim/target_pose", 10);

    _target_sub = nh.subscribe("/dji_sim/target_pose", 10, &JSControl::target_callback, this);
    _js_sub = nh.subscribe("/dji_sim/joy", 10, &JSControl::js_callback, this);

    _init_time = true;
    _freq = 100.0;
}

void JSControl::js_callback(const sensor_msgs::Joy &msg)
{
    _x_rate   = _MAX_X_RATE * msg.axes[3];
    _y_rate   = _MAX_Y_RATE * msg.axes[2];
    _z_rate   = _MAX_Z_RATE * msg.axes[1];
    _yaw_rate = _MAX_YAW_RATE * msg.axes[0];
}

void JSControl::publish_target(const ros::TimerEvent& event)
{
    update_time();

    tf::Vector3 delta_position_global, delta_position_local;
    delta_position_local.setValue(_x_rate * _dt, _y_rate * _dt, _z_rate * _dt);
    delta_position_global = _target_rotation * delta_position_local;

    _target_position[0] += delta_position_global[0];
    _target_position[1] += delta_position_global[1];
    _target_position[2] += delta_position_global[2];

    _target_orientation[2] += _yaw_rate * _dt;

    _target_quaternion.setRPY(_target_orientation[0], _target_orientation[1], _target_orientation[2]);


    geometry_msgs::Pose msg;
    msg.orientation.x = _target_quaternion.x();
    msg.orientation.y = _target_quaternion.y();
    msg.orientation.z = _target_quaternion.z();
    msg.orientation.w = _target_quaternion.w();

    msg.position.x = _target_position.x();
    msg.position.y = _target_position.y();
    msg.position.z = _target_position.z();

    _target_pub.publish(msg);

    ROS_INFO_ONCE("Publish target");
}


void JSControl::update_time()
{
    _time = ros::Time::now().toSec();

    if(_init_time) {
        _dt = 1.0 / _freq;
        _init_time = false;
    }
    else {
        _dt = _time - _time_old;
    }
    _time_old = _time;

    if(fabs(_dt) > 1.5 * 1.0 / _freq || fabs(_dt) < 0.7 * 1.0 / _freq) {
        _dt = 1.0 / _freq;
    }
}
void JSControl::target_callback(const geometry_msgs::Pose &msg)
{
    _target_position[0] = msg.position.x;
    _target_position[1] = msg.position.y;
    _target_position[2] = msg.position.z;

    tf::quaternionMsgToTF(msg.orientation, _target_quaternion);

    _target_rotation.setRotation(_target_quaternion);
    _target_rotation.getRPY(_target_orientation[0],_target_orientation[1],_target_orientation[2]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "js_control_node");
    ros::NodeHandle n("~");

    JSControl joy(n);
    ros::Timer timer = n.createTimer(ros::Duration(0.01), &JSControl::publish_target, &joy);
    ros::spin();

    return 0;
}

