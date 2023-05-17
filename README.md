# ASV ROS package
### Install
Create the `ares_ros_ws` folder as workspace, in the  home directory (need to satisfy some path requirements). Copy in the `src` folder the `asv` package. 

This version is made for real life application on an omnidirectional surface vehicle equipped with a C099-F9P GPS receiver and a VN-100 IMU. 
Once the control parameters have been set, run `roslaunch asv asv_core.launch`, enable the controller and the command governor module by running `rosrun asv enable_control.py` and `rosrun asv enable_cg.py` 
