# Log Book
### Here I will take note of every progress and trouble I had while writing this code


## Log 1: Getting started
Assuming you have already installed [ROS 2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) and [PX4-Autopilot](https://docs.px4.io/main/en/ros2/user_guide.html), we need to create a folder to our work

```
mkdir -p ~/name_of_the_folder/src/
cd ~/name_of_the_folder/src/
```

then clone the px4_msgs to the **/src** directory

```
git clone https://github.com/PX4/px4_msgs.git
```

and source the **ROS 2 environment** into our project

```
cd ..
source /opt/ros/humble/setup.bash
colcon build
```
