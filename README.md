# EKF-Sensor Fusion

This is a multi-sensor error-state EKF implementation for localization of a UAV.
More information on usage and Algorithm description can be found on the [wiki](https://github.com/larics/sensor_fusion/wiki)

## Install
Classic catkin project build:
Then:
```
cd <path_to_catkin_ws>/src
git clone https://github.com/larics/sensor_fusion.git
catkin build
```

You might also need to install yaml-cpp from:
https://github.com/jbeder/yaml-cpp


## Configuration file

This is where the user configures the filter, detailed instructions can be found on the wiki at
[Using the package](https://github.com/larics/sensor_fusion/wiki/Using-the-package)

## Run the test
in the first terminal window run:
```
roslaunch sensor_fusion sensor_fusion.launch
```
then in the second:
```
cd bags/
rosbag play tent_test_cam_posix_2020-11-20-11-50-06.bag
```

now you can plot the output state (/es_ekf/odom) using plot juggler or some other visualization tool.

Testing ssh
