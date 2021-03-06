# Installing ROS Indigo on the Raspberry Pi 3 Model B

This is the documentation/instruction of my personal experience installing ROS Indigo from source on my Raspberry Pi 3 Model B. Most of the instructions are following [this ROS document](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi). However, some of the troubleshooting solutions are referring from [http://www.infocool.net/kb/ASP/201612/249064.html](http://www.infocool.net/kb/ASP/201612/249064.html).

it's not success yet, I will keep updating this documentation.

The instructions that additional to [this ROS document](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi) will be mark like this :warning:.


## 1. Prerequisites
#### 1.1 - Check OS Image that running on My Pi
Raspbian Jessie is being used as the OS on my Raspberry Pi 3. The download page for current images of Raspbian is [http://www.raspberrypi.org/downloads/](http://www.raspberrypi.org/downloads/).

If you would like to see what OS image you are running on your Raspberry Pi, you can use this command in Pi's terminal,
```
cat /etc/os-release
```
Here is the result of my Raspberry Pi,
```
PRETTY_NAME="Raspbian GNU/Linux 8 (jessie)"
NAME="Raspbian GNU/Linux"
VERSION_ID="8"
VERSION="8 (jessie)"
ID=raspbian
ID_LIKE=debian
HOME_URL="http://www.raspbian.org/"
SUPPORT_URL="http://www.raspbian.org/RaspbianForums"
BUG_REPORT_URL="http://www.raspbian.org/RaspbianBugs"
```


#### 1.2 - Setup ROS Repositories
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu jessie main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
```
Then, make sure the Debian package index is up-to-date:
```
$ sudo apt-get update
$ sudo apt-get upgrade
```

#### 1.3 - Install Bootstrap Dependencies
```
$ sudo apt-get install python-pip python-setuptools python-yaml python-distribute python-docutils python-dateutil python-six
$ sudo pip install rosdep rosinstall_generator wstool rosinstall
```

#### 1.4 - Initializing rosdep
```
$ sudo rosdep init
$ rosdep update
```


## 2. Installation

#### 2.1 - Create a catkin Workspace

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


#### 2.2 - Resolve Dependencies

Before we can build the catkin workspace, there are some missing dependencies we need to manually build first. The required packages can be built from source in a new directory:
```
$ mkdir ~/ros_catkin_ws/external_src
$ sudo apt-get install checkinstall cmake
$ sudo sh -c 'echo "deb-src http://mirrordirector.raspbian.org/raspbian/ testing main contrib non-free rpi" >> /etc/apt/sources.list'
$ sudo apt-get update
```

Since I choose to install ```Desktop``` version of ROS, the following packages are needed. Please note, the packages installation order matters.

**libconsole-bridge-dev:**
```
$ cd ~/ros_catkin_ws/external_src
$ sudo apt-get build-dep console-bridge
$ apt-get source -b console-bridge
$ sudo dpkg -i libconsole-bridge0.2*.deb libconsole-bridge-dev_*.deb
```

**liblz4-dev:**
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
  
  **Note:** recent version of urdfdom_headers are incompatible with the ROS Indigo because of changes from using Boost shared pointers to C++11 shared pointers. '9aed725' is the commit hash for the version just prior to the shared pointer change.
  
  :warning: **Also note:** this '9aed725' version doesn't include utils.h file. This will cause problem for the next package installation, so we need to add this utils.h file manually as using following command,
```
$ cd /usr/local/include/urdf_model
$ sudo wget https://raw.githubusercontent.com/ros/urdfdom_headers/master/urdf_model/include/urdf_model/utils.h
```
  
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


#### 2.3 - Resolving Dependencies with rosdep
```
$ cd ~/ros_catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:jessie
```


#### 2.4 - Two Patches
There are two patches we have to do in order to build the catkin Workspace successfully. One for collada_urdf as described [here](https://github.com/ros/robot_model/issues/12). Another patch is for rviz's mesh_loader. Since the instruction of how to apply those patch is seperated in several places (this might confuse and you might feel frustrated to find all the necessary info all at once), I will document the solution that work for me bellow,

**Apply collada_urdf Patch**

```
$ cd ~/ros_catkin_ws/src/robot_model/collada_urdf/src
```

Then, download the patch file by visiting [http://groups.google.com/group/ros-sig-embedded/attach/1708811e0359ec39/0001-fixed-arm-build.patch?part=0.1&authuser=0&view=1](http://groups.google.com/group/ros-sig-embedded/attach/1708811e0359ec39/0001-fixed-arm-build.patch?part=0.1&authuser=0&view=1), and rename it to fix.patch.

```
$ patch < fix.patch
```

**Apply rviz's mesh_loader Patch**

```
$ nano ~/ros_catkin_ws/src/rviz/src/rviz/mesh_loader.cpp
```

Then, adding the following code at the end of all ```#include```,

```cpp
#ifdef __arm__  
#include <strings.h>
bool Assimp::IOSystem::ComparePaths(const char *p1, const char *p2) const  
{  
  return !::strcasecmp(p1,p2);  
}  
#endif
```

:warning:**Apply urdf's model Patch**
```
$ nano ~/ros_catkin_ws/src/robot_model/urdf/CMakeLists.txt
```

Then, adding the following code bellow ``project(urdf)```,

```cmake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
```


These patches are actually do the samething. However, I didn't find a patch file specifically for the rviz's mesh_loader from internet. So, I just manually adding code. I will construct a patch file for it once I figure out how to write a patch.


#### 2.5 - Building the catkin Workspace
```
$ sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo
```

This will take quite a long time. Since I fail a couple times, I'm not sure exactly how long does it take to finish. It's roughly 1 or 2 hours, maybe. But, after it finished, ROS should be installed!

Remember to source the new installation:
```
$ source /opt/ros/indigo/setup.bash
```
Or optionally source the setup.bash in the ~/.bashrc, so that ROS environment variables are automatically added to your bash session every time a new shell is launched:
```
$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
```

# References
- [http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi)
- [http://www.infocool.net/kb/ASP/201612/249064.html](http://www.infocool.net/kb/ASP/201612/249064.html)
- [https://github.com/ros/robot_model/issues/12](https://github.com/ros/robot_model/issues/12)
- [https://groups.google.com/d/msg/ros-sig-embedded/26XlDtZhyNs/OexZAx6BCBcJ](https://groups.google.com/d/msg/ros-sig-embedded/26XlDtZhyNs/OexZAx6BCBcJ)
