# Installing ROS Indigo on the Raspberry Pi 3 Model B

This is the documentation/instruction of my personal experience installing ROS Indigo from source on my Raspberry Pi 3 Model B. Most of the instructions are following [this ROS document](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi). However, some of the troubleshooting solutions are referring from [http://www.infocool.net/kb/ASP/201612/249064.html](http://www.infocool.net/kb/ASP/201612/249064.html).

it's not success yet, I will keep updating this documentation.


## Prerequisites
#### OS on Pi
Raspbian Jessie is being used as the OS on my Raspberry Pi 3. The download page for current images of Raspbian is [http://www.raspberrypi.org/downloads/](http://www.raspberrypi.org/downloads/).

If you would like to see what OS image you are running on your Raspberry Pi, you can use this command in Pi's terminal,
```
cat /etc/os-release
```
Here is the result of my Raspberry Pi,
> To be update


#### Setup ROS Repositories
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu jessie main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
```
Then, make sure the Debian package index is up-to-date:
```
$ sudo apt-get update
$ sudo apt-get upgrade
```

#### Install Bootstrap Dependencies
```
$ sudo apt-get install python-pip python-setuptools python-yaml python-distribute python-docutils python-dateutil python-six
$ sudo pip install rosdep rosinstall_generator wstool rosinstall
```

#### Initializing rosdep
```
$ sudo rosdep init
$ rosdep update
```


## Installation
#### Create a catkin Workspace
In order to build the core packages, we will need a catkin workspace. Create one now:
```
$ mkdir ~/ros_catkin_ws
$ cd ~/ros_catkin_ws
```

According to [this ROS document](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi), there are two versions of ROS package you can choose to install. I want the ```Desktop``` one, so I use the command below:
```
$ rosinstall_generator desktop --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-desktop-wet.rosinstall
$ wstool init src indigo-desktop-wet.rosinstall
```

The command will take a few minutes to run.





