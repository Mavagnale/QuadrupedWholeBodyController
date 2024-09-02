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
```
cd lib/idyntree
mkdir build && cd build
$ cmake -DCMAKE_INSTALL_PREFIX=<prefix> <additional_platform_specific_options> ..
$ make
$ make install
```
### Install ros_control
`sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers`
### Compile package with catkin_make
```
roscd
cd ..
catkin_make
```
