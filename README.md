# README #

A simple gazebo simulator for inspection project. Currently robot model follows the DJI inspection setup (with g2, imu and gimbal camera). 


### How do I get set up? ###
1 Install dependencies of gazebo and joy.
```
sudo apt-get install ros-indigo-joy ros-indigo-gazebo*
```
2 Open a new terminal, start gazebo server and client. This should load the DJI matrice robot model and a bridge model:
```
roslaunch dji_gazebo dji.launch
```

3 Open another new terminal, start g2_control. This step makes g2 spinning and publish point cloud in robot body frame:
```
roslaunch g2_control g2_control.launch
```

4 Open a third terminal, start robot control node. This makes the robot listens to a ```/dji_sim/target_pose``` topic and is controlled to reach the goal:
```
roslaunch robot_control robot_control.launch
```
Now if you have a joystick, you can drive the robot now. Make sure to adjust the device name in ```robot_control/launch/js_control.launch```. Alternatively you can publish your topic using 

```
rostopic pub -r 10 /dji_sim/target_pose geometry_msgs/Pose "{position: {z: 1.0}, orientation: {w: 1.0}}"
```

5 Open the fourth terminal to start localization. 
```
roslaunch lidar_ekf dji_gazebo.launch 
```
6 By default, gimbal stays in zero configuration. It listens to ```/dji_sim/gimbal_target``` such that gimbal orientation is controlled in robot body frame.

```
roslaunch gimbal_control gimbal_control.launch
```
7 You should see the simulation like following figure.
![dji_sim.png](https://bitbucket.org/repo/gBoX7x/images/985936132-dji_sim.png)

### Who do I talk to? ###

* Weikun Zhen (zhenwk@gmail.com)


### License ###
[This software is BSD licensed.](http://opensource.org/licenses/BSD-3-Clause)

Copyright (c) 2017, Carnegie Mellon University
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.