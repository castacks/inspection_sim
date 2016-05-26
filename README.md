# README #

A simple gazebo simulator for inspection project. Currently robot model follows the DJI inspection setup (with g2, imu and gimbal camera). 


### How do I get set up? ###

1 Open a new terminal, start gazebo server and client. This should load the DJI matrice robot model and a bridge model:
```
roslaunch dji_gazebo dji.launch
```

2 Open another new terminal, start g2_control. This step makes g2 spinning and publish point cloud in robot body frame:
```
roslaunch g2_control g2_control.launch
```

3 Open a third terminal, start robot control node. This makes the robot listens to a ```/dji_sim/target_pose``` topic and is controlled to reach the goal:
```
roslaunch robot_control robot_control.launch
```
Now if you have a play station, you can drive the robot now. Make sure to adjust the device name in ```robot_control/launch/js_control.launch```.

4 Open the fourth terminal to start localization. 
```
roslaunch lidar_ekf dji_gazebo.launch 
```

5 You should see the simulation like following figure.
![dji_sim.png](https://bitbucket.org/repo/gBoX7x/images/985936132-dji_sim.png)

### Who do I talk to? ###

* Weikun Zhen (zhenwk@gmail.com)