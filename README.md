# EKF-Sensor Fusion

This is a EKF implementation with n sensor fusion for localization of a UAV.

## Install
Classic catkin project build, first follow instructions here:
https://github.com/larics/mmuav_gazebo
Then:
```
cd <path_to_catkin_ws>/src
git clone https://github.com/larics/sensor_fusion.git
catkin build
```

You might also need to install yaml-cpp from:
https://github.com/jbeder/yaml-cpp


## Configuration file

In the config folder there is a configuration file ```config.yaml``` .
This is an example config file with 2 sensors with a short explanation for each param.
Adding more sensor should be seamless.
```
#Vehicle params

initial_state: [0,0,0,0,0,0,0,0,0,0,0,0] #initial state of the EKF
mass: 2.33 # mass of UAV
g: 9.81 #gravitational constant

b: 0.000005  #thrust
d: 0.0000806428 #drag
l: 0.314 #arm length (from propeller to center of mass)

#moments of inertia
Ixx: 0.075032
Iyy: 0.075032
Izz: 0.15006

T: 0.02 # set the freq for the EKF
# correlation matrix of the model
Q: 0.001 # How much we trust the model
Qz: 1  #we trust the z axis a bit less than the rest

#define the sensors we are using and give them a prefix
Sensor_prefix: [front,back] 
#front sensor, Give each element a prefix front

# correlation matrix of the sensor (diagonal matrix)
front_R: [0.0025, 0.0025, 0.0025] 
front_odom: 0 #is it an odometry sensor
front_topic: /uav/odometry2 #topic that the sensor publishes to

#back
#give each element a prefic back
back_R: [0.0049, 0.0049, 0.0049] # correlation matrix of the sensor
back_odom: 0
back_topic: /uav/odometry3
```
## Run simulation
 
 Note: if you are using more than one sesnor you should add a sensor (odometry plugin) to the file 
 ```uav.gazebo.xacro``` in the mmuav_gazebo package (https://github.com/larics/mmuav_gazebo)
. There you define the topic and the noise of the sensor.

To run a simulation with a small trajectory run the following commands 
(after building the project and sourcing your setup file).
 
```roslaunch mmuav_gazebo uav_trajectory.launch```

```roslaunch sensor_fusion sensor_fusion```

## Plot

To plot the data go in to the matlab directory and run the script ```plot_path.m```

## Expected output
```
Pose with model
X: 0
Y: 0
Z: 0
Pose with model
X: 0
Y: 0
Z: 0
U1 0
Pose with model
X: 0
Y: 0
Z: 0
Pose with model
X: 0
Y: 0
Z: 0
```
