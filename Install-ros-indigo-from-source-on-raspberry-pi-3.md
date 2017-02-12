# Installing ROS Indigo on the Raspberry Pi 3 Model B

This is the documentation/instruction of my personal experience installing ROS Indigo from source on my Raspberry Pi 3 Model B. Most of the instructions are following [this ROS document](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi). However, some of the troubleshooting solutions are referring from [http://www.infocool.net/kb/ASP/201612/249064.html](http://www.infocool.net/kb/ASP/201612/249064.html).

it's not success yet, I will keep updating this documentation.


## Prerequisites
#### OS Image that running on My Pi
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

#### Resolve Dependencies
Before we can build the catkin workspace, there are some missing dependencies we need to manually build first. The required packages can be built from source in a new directory:
```
$ mkdir ~/ros_catkin_ws/external_src
$ sudo apt-get install checkinstall cmake
$ sudo sh -c 'echo "deb-src http://mirrordirector.raspbian.org/raspbian/ testing main contrib non-free rpi" >> /etc/apt/sources.list'
$ sudo apt-get update
```

Since I choose to install ```Desktop``` version of ROS, the following packages are needed.

** libconsole-bridge-dev:**
```
$ cd ~/ros_catkin_ws/external_src
$ sudo apt-get build-dep console-bridge
$ apt-get source -b console-bridge
$ sudo dpkg -i libconsole-bridge0.2*.deb libconsole-bridge-dev_*.deb
```

** liblz4-dev:**
```
$ cd ~/ros_catkin_ws/external_src
$ apt-get source -b lz4
$ sudo dpkg -i liblz4-*.deb
```

 **liburdfdom-headers-dev:**
```
$ cd ~/ros_catkin_ws/external_src
$ git clone https://github.com/ros/urdfdom_headers.git
$ cd urdfdom_headers
$ git reset --hard 9aed725
$ cmake .
$ sudo checkinstall make install
```
  When check-install asks for any changes, the name (2) needs to change from "urdfdom-headers" to "liburdfdom-headers-dev" otherwise the rosdep install wont find it.
  Note: recent version of urdfdom_headers are incompatible with the ROS Indigo because of changes from using Boost shared pointers to C++11 shared pointers. '9aed725' is the commit hash for the version just prior to the shared pointer change.

 **liburdfdom-dev:**
```
$ cd ~/ros_catkin_ws/external_src
$ sudo apt-get install libboost-test-dev libtinyxml-dev
$ git clone https://github.com/ros/urdfdom.git
$ cd urdfdom
$ cmake .
$ sudo checkinstall make install
```
  When check-install asks for any changes, the name (2) needs to change from "urdfdom" to "liburdfdom-dev" otherwise the rosdep install wont find it.

**collada-dom-dev:**
```
$ cd ~/ros_catkin_ws/external_src
$ sudo apt-get install libboost-filesystem-dev libxml2-dev
$ wget http://downloads.sourceforge.net/project/collada-dom/Collada%20DOM/Collada%20DOM%202.4/collada-dom-2.4.0.tgz
$ tar -xzf collada-dom-2.4.0.tgz
$ cd collada-dom-2.4.0
$ cmake .
$ sudo checkinstall make install
```
  When check-install asks for any changes, the name (2) needs to change from "collada-dom" to "collada-dom-dev" otherwise the rosdep install wont find it.

