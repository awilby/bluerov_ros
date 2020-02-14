# bluerov_ros
Packages for using BlueROV2 platform with the Robot Operating System (ROS).

This package is under development and will change frequently. Use at your own risk. 

## Installation

### Installing ROS 

For using ROS, we recommend running Ubuntu Mate instead of the provided Raspbian images. Please note that if you choose to use Ubuntu Mate you will have to re-configure some settings and software to get the ROV running in the "out of the box" configuration again, so this choice will depend on your needs.

If running the default Raspbian image, follow the instructions on [Installing ROS Kinetic on the Raspberry Pi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi) to compile ROS from source.

If you choose to run Ubuntu Mate, follow the usual [ROS installation instructions](http://wiki.ros.org/kinetic/Installation) for Ubuntu. (Note: this has currently been tested with ROS Kinetic, but it is likely that ROS Melodic will work too without too much headache).

We also recommend installing `catkin_tools`, which includes some nicer build tools: 

`sudo apt-get install python-catkin-tools`


Make sure to create a catkin workspace by following the instructions in [ROS Environment Setup](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).


### Install Dependencies

After installing ROS, also install the following packages (these instructions assume you are using Ubuntu Mate and are installing the packages in the apt repositories; Raspbian users must compile from source): 

`sudo apt-get install ros-kinetic-joy ros-kinetic-robot-state-publisher ros-kinetic-robot-localization ros-kinetic-mavros ros-kinetic-mavros-extras ros-kinetic-sound-play`

Install GeographicLib (needed for mavros) by downloading and running the installation script:

`wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh`


### Install bluerov_ros

In <your_catkin_workspace>/src, clone this repository:

```
git clone --recurse-submodules https://github.com/awilby/bluerov_ros.git
cd ..
catkin build 
```

### Optional Dependencies

There are some optional packages you can install for additional functionality. If you want to perform ekf_localization, install `sudo apt-get install ros-kinetic-robot-localization`. We are using the IMU and viso2 with the front camera to perform localization. 

Viso2 needs to be compiled from source. From <your_catkin_workspace>/src:


```
git clone https://github.com/srv/viso2.git
cd ..
catkin build
```


## Setup


### Install udev rules

Copy the udev rules from the bluerov_robot and bluerov_teleop packages to `/etc/udev/rules.d/`. Reload udev rules by running `sudo udevadm control --reload`.
