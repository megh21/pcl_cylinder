
# object pose detection in pointcloud with PCL

## Requirments

- ROS Melodic
- install and build [ pcl ](https://pointclouds.org/downloads/)
- [pcl_ros](http://wiki.ros.org/pcl_ros) 
```bash
   sudo apt-get update
   sudo apt-get install pcl-tools
   sudo apt-get install ros-melodc-pcl-ros
```


## Getting Started


### Prerequisites

here, you will need:

* **CMake v3.15+** - found at [https://cmake.org/](https://cmake.org/)

* **C++ Compiler** - needs to support at least the **C++17** standard, i.e. *MSVC*,
*GCC*, *Clang*

> ***Note:*** *You also need to be able to provide ***CMake*** a supported
[generator](https://cmake.org/cmake/help/latest/manual/cmake-generators.7.html).*
## Installing and Building the project

It is fairly easy to install the project, all you need to do is clone if from
[GitHub](https://github.com/megh21/pcl_cylinder) or

(also on **GitHub**).


To install an already built project, you need to run the `install` target with CMake.
For example:

```bash
cd build
cmake ..
make
# not available for all yet
make install 
# OR
# cmake --build build --target install --config Release
# # a more general syntax for that command is:
# cmake --build <build_directory> --target install --config <desired_confi*g>
```

> ***Note***: **you have to go in individual folders and have to build everything by yourself**

## Running the tests
go to each folders build\\devel\\lib\\_file_name__\

```bash
./filename
```
### Use zed2i 
 - requires [zed sdk](https://www.stereolabs.com/developers/release/)
 - requires [zed ros wrapper](https://github.com/stereolabs/zed-ros-wrapper)
  
```sh
roslaunch zed_wrapper zed2i.launch 
```

### Use rviz
use rviz to visualize the topic pointcloud2, topic name etc.

### Use PCD file

Example: `./samples/*.pcd`

```bash
pcl_viewer *.pcd
```

## Contributing

Please while committing, **do not commit build folders**. 

```bash
# cd into each build folder
cd build 
rm * -rf
cd ..
# git commit git push 

```


## Authors

* **Meghraj Bagde** - [@megh21](https://github.com/megh21)
