# Installation
### Clone the repository
`git clone https://github.com/Mavagnale/QuadrupedWholeBodyController.git`
### Update the submodules
`git submodule update --recursive --init`
### Install Eigen
Install Eigen by copying the Eigen folder downloaded from [here](http://eigen.tuxfamily.org/index.php?title=Main_Page#Download) into /usr/local/include
### Install qpOASES
```
cd lib/qpOASES
make
```

### Install iDynTree
Install iDynTree dependencies 
```
sudo apt-get install build-essential libeigen3-dev libxml2-dev coinor-libipopt-dev libassimp-dev libirrlicht-dev libglfw3-dev
```
```
cd lib/idyntree
mkdir build && cd build
$ cmake -DCMAKE_INSTALL_PREFIX=<prefix> <additional_platform_specific_options> ..
$ make
$ make install
```
### Install gazebo and ros_control
`sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-ros-control ros-noetic-ros-controllers`
### Compile package with catkin_make
```
roscd
cd ..
catkin_make
```
