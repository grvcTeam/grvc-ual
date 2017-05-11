Making this package work may be a little bit tricky, follow this steps and hail the gods of gazebo and ros: 

1. At your ros workspace: git clone --recursive https://github.com/PX4/Firmware.git
2. Go to http://dev.px4.io/starting-installing-linux.html#installation and follow the section Installation instructions
3. At ros workspace: catkin_make
4. At Firmware: make
5. At Firmware: make posix_sitl_default gazebo

You should be seeing a sitl simulation in gazebo, but this is not what we want, kill it and then...

6. roslaunch px4_bringup test_simulate.launch
