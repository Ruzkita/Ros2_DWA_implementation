# Log Book
### Here I will take note of every progress and trouble I had while writing this code. I'll divide this into two kinds os Logs (info and problem)

<details>
<summary>INFO LOGS</summary>

  
## Info Log 1: Getting started
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


## Info Log 2: Publishers, subscribers and the timer
If I get it right, the publishers and subscribers in the PX4-Ros2 interaction work as a comunication with the drone. As the publishers are used to transmit info and commands for the drone, the subscribers are used to receive info about the drone and the external environment. So, if you want to command your vehicle you need to create a publisher and if you want to receive info from your vehicle you need to create a subscriber.

# Creating a **publisher**:
Every publisher will follow this structure

```
NODE.create_publisher(msg_type, topic, qos_profile)
```

using the VehicleCommand.msg from the drone_control.py we will have

```
self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
```

# Creating a **subscriber**:
The subscriber creation follow almost the same structure

```
NODE.create_subscription(msg_type, topic, callback, qos_profile)
```
and using the VehicleStatus.msg as an example

```
self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
```

# Timer:
The timer is a function that will be called every x seconds. Your structure is
```
.create_timer(timer_period_sec, callback)
```

for example
```
self.timer = self.create_timer(0.1, self.timer_callback)
```

This three functions will be the base for the comunication with the vehicle in every code.

</details>

<details>
<summary>PROBLEM LOGS</summary>
  
## Problem Log 1: VehicleCommand - SOLVED
I need the VehicleCommand.msg message from PX4-Autopilot but i don't know how it works exactly and in the repository there is not enought info about this.

**Edit:** I almost understand the Vehicle Command now and I can make the drone arm, but still receiving some error messages about the "command" variable.

**Edit2:** After I set a trajectory the vehicle stop showing errors.

</details>


