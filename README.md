# ROS Teleops for Pepper

Instructions for setting up teleoperations for Pepper through ROS noetic. 
The ROS naoqi teleoperations official documentation can be found [here](http://wiki.ros.org/nao_teleop), however it is not available for ROS noetic.
To work around this the package must be launched locally. 
This repository includes the modified code to be run locally and instructions on how to do this.

**Note this tutorial uses a Logitech F170 controller.**

**Pre-Requisite:** The catkin workspace and Naoqi Driver should be set up. This can be done by following the `Direct Installation` steps found in [ROS to NAO Bridge Setup Instructions](https://github.com/rosielab/ROStoNAO-Bridge-Docker-Setup).

## Setup
### 1. Set Up Controller 

More information can be found [here](https://github.com/husarion/logitech_f710_ros).

#### Connecting the Controller 
Ensure the device is in **DirectInput** mode. This is done by sliding the switch on the front of the controller to the **D**.

Click on the Logitech button in the centre of the controller. When the greenlight on the controller begins flashing, plug the USB connector into the computer. 

You may need to install the Xbox gamepad driver. This is done using:
```
sudo apt install xboxdrv
```

#### Testing the Connection 

To test the connection, open a terminal window and install:
```
sudo apt install joystick
```

The connection can then be tested by running:
```
jstest /dev/input/js0
```

You should see values of the controller's joystick axes and values for the buttons (off/on).

### 2. Download Teleops package 

Download this ROS teleops package and place it in yours `catkin_ws/src` folder.

### 3. Install dependancies 
Install the following dependencies:
```
sudo apt install ros-noetic-roscpp
sudo apt install ros-noetic-std-msgs
sudo apt install ros-noetic-std_srvs
sudo apt install ros-noetic-naoqi-bridge-msgs
sudo apt install ros-noetic-sensor-msgs
sudo apt install ros-noetic-actionlib
sudo apt install ros-noetic-joy
```

Note: These dependancies can be found listed in package.xml

### 4. Prepare Pepper 

Connect Pepper and your computer to the same network (NETGEAR23-5G).

SSH into Pepper to turn off Automated Life:
```
ssh nao@<naoip>
qicli call ALAutonomousLife.setState disabled
qicli call ALMotion.wakeUp
```

### 5. Launch Naoqi Driver
In a new terminal window, source ROS setup:
```
cd ~/catkin_ws/
source /opt/ros/noetic/setup.bash
catkin_make
source ./devel/setup.bash
```

Then run:
```
roslaunch naoqi_driver naoqi_driver.launch nao_ip:=<nao_ip> nao_port:=<nao_port> roscore_ip:=<roscore_ip> network_interface:=<network_interface> username:=<username> password:=<password>
```

For example this could be:
```
roslaunch naoqi_driver naoqi_driver.launch nao_ip:=10.0.0.4 nao_port:=9503 roscore_ip:=127.0.0.1 network_interface:=docker0 username:=nao password:=nao
```
### 6. Launch Teleops
In a new terminal window, source ROS setup and build project:
```
cd ~/catkin_ws/
source /opt/ros/noetic/setup.bash
catkin_make
source ./devel/setup.bash
```

Then run: 
```
roslaunch nao_teleop teleop_joy.launch
```

You should now receive the message:
```
starting button is set to 9
```

## Controlling Pepper

Click the `start` button on the controller and Pepper should now say "Gamepad control enabled". Clicking the `start` button again will disable the controller. You may toggle between these states. The controller must be **set to enabled to control Pepper** through it.

### Moving Pepper
* Use the `left joystick` to turn Pepper's orientation.
* Use the `right joystick` to move Pepper in the chosen direction.

### Moving Pepper's Head
Press and hold the `RB` button and...
* Use the `left joystick` to turn Pepper's head from side to side.
* Use the `right joystick` to move Pepper's head up and down.

### Pepper Speech
This package has set a few predetermined phrases for Pepper to say:
* Press the `B` button for Pepper to say "Hello".
* Press the `X` button for Pepper to say "Goodbye".
* Press the `Y` button for Pepper to say "Excuse me".
* Press the `A` button for Pepper to say "Thank you".

### Poses - Deprecated


The original package had mapped certain buttons to poses for Pepper to perform, but they do not currently work. 

This is because the [Naoqi Pose](http://wiki.ros.org/naoqi_pose) node must be running. The code for this can be found [here](https://github.com/ros-naoqi/naoqi_bridge/tree/master/naoqi_pose), however it is deprecated. 

Because of this, any uses of this node **have been commented out**, but have not been deleted in case this changes in the future. 

A partially updated version of the Naoqi Pose Manager can be found [here](https://github.com/alyssarusk/naoqi-pose/tree/main). This package also requires Naoqi Node found in Naoqi Driver Py package (also deprecated). A partially updated version of the Naoqi Node can be found [here](https://github.com/alyssarusk/naoqi-driver-py/tree/main)











