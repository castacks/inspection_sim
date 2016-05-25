# README #

A simple gazebo simulator for inspection project. Currently robot model follows the DJI inspection setup (with g2, imu and gimbal camera). 


### How do I get set up? ###

* Open a new terminal, start gazebo server and client. This should load the DJI matrice robot model and a bridge model:
```
roslaunch dji_gazebo dji.launch
```

* Open another new terminal, start g2_control. This step makes g2 spinning and publish point cloud in robot body frame:
```
roslaunch g2_control g2_control.launch
```

* Open a third terminal, start robot control node. This makes the robot listens to a ```/dji_sim/target_pose``` topic and is controlled to reach the goal:
```
roslaunch robot_control robot_control.launch
```
* Also if you have a play station, you can drive the robot now.


### Who do I talk to? ###

* Weikun Zhen (zhenwk@gmail.com)