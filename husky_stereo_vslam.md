# Husky + OAK-D Visual SLAM Stack Manual

## Pre-requisites

We assume you have a working ROS Noetic environment on your Jetson AGX Orin. 

Install the required packages for dependency:
```bash
cd sep799-sidewalk/catkin_ws
rosdep check --from-paths src --ignore-src -r # Check the list to be installed
rosdep install --from-paths src --ignore-src -r -y # Install the packages without prompting
```

To note, the standalone Jetson AGX Orin was installed with Ubuntu 20.04 and a source-code compiled OpenCV, but we failed to build rtabmap_ros with that OpenCV version. Therefore, we used dockerized rtabmap_ros image and joined the ROS Noetic environment using the following command:

```bash
sudo -s # Switch to root user

XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run -it --rm \
   --privileged \
   -e DISPLAY=$DISPLAY \
   -e QT_X11_NO_MITSHM=1 \
   -e NVIDIA_VISIBLE_DEVICES=all \
   -e NVIDIA_DRIVER_CAPABILITIES=all \
   -e XAUTHORITY=$XAUTH \
   --runtime=nvidia \
   --network host \
   -v $XAUTH:$XAUTH \
   -v /tmp/.X11-unix:/tmp/.X11-unix \
   -v /home/sidewalk/Developer/sep799-sidewalk/catkin_ws:/root/catkin_ws \
   -v /home/sidewalk/.ros/rtabmap:/root/.ros \
   -e LD_LIBRARY_PATH=/opt/ros/noetic/lib:/opt/ros/noetic/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu/tegra \
    introlab3it/rtabmap_ros:noetic-latest \
    /bin/bash
```

After entering the bash:
```bash
unset ROS_NAMESPACE
cd /root/catkin_ws/
rm -rf build/ devel/
catkin_make
source devel/setup.bash
export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1:$LD_PRELOAD
apt-get update --allow-insecure-repositories
apt install ros-noetic-navigation ros-noetic-husky-navigation --allow-unauthenticated -y # GlobalPlanner is required for move_base to work properly
```

This will set up a dockerized rtabmap_ros bash environment with display graphics support on the host machine. 

Change `sidewalk` to your own username accordingly.

## Launching the Stack

### One Extra Step for Simulation

If you are using the simulation environment, follow these steps to launch the gazebo, which simulates the Husky robot. This step is not nessary if you are using a real device.

```bash
export HUSKY_URDF_EXTRAS=$HOME/Developer/sep799-sidewalk/catkin_ws/src/husky_oakd_stereo_nav/urdf/mcm07_customization.urdf.xacro
roslaunch husky_gazebo husky_playpen.launch
```

The sequence is important! Launch Gazebo first then launch following commands in different terminals.

### Launching the Stack

Our launch file has a naming convention: `{environment}_{sensors}_{vslam_type}.launch`:
- `environment`: `sim` for simulation, `real` for real device; real device comes with depthai_examples `stereo_inertial_node.launch` included 
- `sensors`: `rgbd` for only RGB-D perception, `rgdb_lidar` for RGB-D + LiDAR perception, `stereo` for stereo perception
- `vslam_type`: `slam` for SLAM, `mapping` for mapping only, `navigation` for navigation based on existing maps

Under your **rtabmap_ros installed** terminal and run the following command:
```bash
roslaunch husky_oakd_stereo_nav <launch_file>.launch
```

Deprecated: `rgbd_vslam_no_visual_odom.launch` is created in 2025 Winter term and no longer used. 

### To View the Robot in RViz

Open a new terminal and run the following command:

```bash
export HUSKY_URDF_EXTRAS=$HOME/Developer/sep799-sidewalk/catkin_ws/src/husky_oakd_stereo_nav/urdf/mcm07_customization.urdf.xacro
roslaunch husky_viz view_robot.launch
```

You might need to specify some topics in RViz to visualize the robot properly.

---

### Map Saving

After finish mapping:

```bash
cd catkin_ws/src/husky_oakd_stereo_nav/maps
bash record_maps.bash
```

### Navigation using existing maps

1. Copy desired `.db` file to `~/.ros/rtabmap/rtabmap.db`
2. Specify `static_map_path` in correspondent launch file. 
3. Manually drive the robot around until it has localized in the map
4. Send desired navigation goals using the 2D Nav Goal button in RViz