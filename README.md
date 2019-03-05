# lidar depth

## Installation

### Requirements

In any case:

* png++: 
```shell
 sudo apt-get install libpng++-dev
 ```
* install ros: 
  - follow the instructions on [https://wiki.ros.org/kinetic/Installation](https://wiki.ros.org/kinetic/Installation).
  - you will need to install ros-full (for pcl).
  - don't forget to source your ~/.bashrc afterwards.
* install catkin_tools: 
```shell 
sudo apt-get install python-catkin-tools
 ```
* install opencv_apps: 
```shell
sudo apt-get install ros-kinetic-opencv-apps
```
* install git: 
```shell
sudo apt-get install git
```

### Build

* initiate a catkin workspace:
    ```shell 
    cd ${your_catkin_workspace}
    mkdir ${your_catkin_workspace}/src
    catkin init
    ```

* clone into src of workspace:
    ```shell
    cd ${your_catkin_workspace}/src
    git clone git@github.com:zarmomin/mono_lidar_depth.git
    ```

* clone dependencies and build repos
    ```shell 
    cd ${your_catkin_workspace}/src/mono_lidar_depth
    bash install_dependencies.sh
    catkin build mono_lidar_depth
    ```
