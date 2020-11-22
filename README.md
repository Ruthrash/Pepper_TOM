# Pepper and Theory of Mind Experiments 


## Description

Contains simulation and pereception software packages used for conducting experiments in simulation.

## Dependencies 
- ROS(Melodic)
- Gazebo9 
- CUDA(for YOLO and openPose)
- CUDNN(for Openpose and face recognition)
- [Python Face Recognition](https://pypi.org/project/face-recognition/)


## Usage

Assuming you have installed ROS(and gazebo) and that you are using an Ubuntu machine

```
mkdir -p ~/pepper_ws/src
cd ~/pepper_ws/src 
catkin_init_workspace
git clone --recurse-submodules https://github.com/Ruthrash/Pepper_TOM
```

Building OpenPose- Follow [installation instructions](https://github.com/tramper2/openpose/blob/master/doc/installation.md)  or,

```
cd ~/pepper_ws/src/Pepper_and_TOM/openpose
mkdir build 
cd build 
cmake ../
make -j`nproc`
sudo make install
```


Building ROS packages 

```
cd ~/pepper_ws
catkin_make_isolated 
```

Running Simulation

```
roslaunch pepper_gazebo_plugin pepper_gazebo.launch
```



