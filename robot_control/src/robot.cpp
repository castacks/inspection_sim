#include <robot_control/robot.h>

template <typename T> double sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

RobotControl::RobotControl(ros::NodeHandle &nh)
{
    nh.param("mass",        _mass,          1.0);
    nh.param("inertia",     _inertia,       1.0);
    nh.param("freq",        _freq,          100.0);
    nh.param("max_force",   _MAX_FORCE,     1.0);
    nh.param("max_torque",  _MAX_TORQUE,    1.0);
    nh.param("pos_p",       _pos_p,         1.0);
    nh.param("pos_i",       _pos_i,         0.001);
    nh.param("pos_d",       _pos_d,         3.0);
    nh.param("ori_p",       _ori_p,         1.0);
    nh.param("ori_i",       _ori_i,         0.0);
    nh.param("ori_d",       _ori_d,         1.0);


    _init_time = true;
    _init_ctrl = true;

    _target_position.setZero();
    _target_orientation.setZero();

    _linear_acceleration.setZero();
    _angular_acceleration.setZero();

    _linear_velocity.setZero();
    _angular_velocity.setZero();

    _pos_err_i.setZero();
    _ori_err_i.setZero();

    _pose_sub = nh.subscribe("/gazebo/model_states", 10, &RobotControl::pose_callback, this);
    _target_sub = nh.subscribe("/dji_sim/target_pose", 100, &RobotControl::target_callback, this);

    _model_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
}

void RobotControl::target_callback(const geometry_msgs::Pose::Ptr msg)
{
    _target_position[0] = msg->position.x;
    _target_position[1] = msg->position.y;
    _target_position[2] = msg->position.z;

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation,q);
    tf::Matrix3x3(q).getRPY(_target_orientation[0],_target_orientation[1],_target_orientation[2]);

}

void RobotControl::pose_callback(const gazebo_msgs::ModelStates::Ptr &msg)
{
    for(int i = 0; i < msg->name.size(); i++) {
        if(msg->name[i].compare("dji") == 0) {
            _position[0] = msg->pose[i].position.x;
            _position[1] = msg->pose[i].position.y;
            _position[2] = msg->pose[i].position.z;

            tf::Quaternion q;
            tf::quaternionMsgToTF(msg->pose[i].orientation, q);
            tf::Matrix3x3(q).getRPY(_orientation[0],_orientation[1],_orientation[2]);

            update_time();
            update_control();
            update_state();
            publish_state();
        }
    }

}
void RobotControl::update_time()
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

void RobotControl::update_control()
{
    // error
    _pos_err = _target_position    - _position;
    _ori_err = _target_orientation - _orientation;

    // derivative of error
    if(_init_ctrl) {
        _pos_err_d.setZero();
        _ori_err_d.setZero();
        _init_ctrl = false;
    }
    else {
        _pos_err_d = (_pos_err - _pos_err_prev) / _dt;
        _ori_err_d = (_ori_err - _ori_err_prev) / _dt;
    }

    // integral of error
    _pos_err_i = _pos_err_i + _pos_err;
    _ori_err_i = _ori_err_i + _ori_err;

    // update prevous err
    _pos_err_prev = _pos_err;
    _ori_err_prev = _ori_err;

    // PID control
    _force  = _pos_err * _pos_p + _pos_i * _pos_err_i + _pos_d * _pos_err_d;
    _torque = _ori_err * _ori_p + _ori_i * _ori_err_i + _ori_d * _ori_err_d;


    // bound control input
    if(fabs(_force[0]) > _MAX_FORCE) _force[0] = sgn(_force[0]) * _MAX_FORCE;
    if(fabs(_force[1]) > _MAX_FORCE) _force[1] = sgn(_force[1]) * _MAX_FORCE;
    if(fabs(_force[2]) > _MAX_FORCE) _force[2] = sgn(_force[2]) * _MAX_FORCE;

    if(fabs(_torque[0]) > _MAX_TORQUE) _torque[0] = sgn(_torque[0]) * _MAX_TORQUE;
    if(fabs(_torque[1]) > _MAX_TORQUE) _torque[1] = sgn(_torque[1]) * _MAX_TORQUE;
    if(fabs(_torque[2]) > _MAX_TORQUE) _torque[2] = sgn(_torque[2]) * _MAX_TORQUE;

}

void RobotControl::update_state()
{
    _linear_acceleration  = _force  / _mass;
    _angular_acceleration = _torque / _inertia;

    _linear_velocity  = _linear_velocity  + _linear_acceleration  * _dt;
    _angular_velocity = _angular_velocity + _angular_acceleration * _dt;

    _position    = _position    + _linear_velocity  * _dt;
    _orientation = _orientation + _angular_velocity * _dt;

}

void RobotControl::publish_state()
{
    gazebo_msgs::ModelState msg;
    msg.model_name = "dji";
    msg.pose.position.x = _position.x();
    msg.pose.position.y = _position.y();
    msg.pose.position.z = _position.z();

    tf::Quaternion q;
    q.setRPY(_orientation[0], _orientation[1], _orientation[2]);
    msg.pose.orientation.w = q.w();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();

    _model_pub.publish(msg);
}

