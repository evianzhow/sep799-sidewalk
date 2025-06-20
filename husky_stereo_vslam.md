# Husky + OAK-D Navigation Stack using dockerized rtabmap_ros

The sequence is important! Launch Gazebo first then rtabmap_ros

Terminal 1:
```bash
export HUSKY_URDF_EXTRAS=$HOME/Developer/sep799-sidewalk/catkin_ws/src/husky_oakd_stereo_nav/urdf/mcm07_customization.urdf.xacro
roslaunch husky_gazebo husky_playpen.launch
```

Terminal 2:
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
roslaunch husky_oakd_stereo_nav rgbd_vslam_no_visual_odom.launch
```

If you are using a real device, launch husky_oakd_stereo_nav with following command:
```bash
roslaunch husky_oakd_stereo_nav rgbd_vslam_no_visual_odom.launch oak_d_type:=device
```

`imu_filter_madgwick` should be automatically launched as a node.

Terminal 3:
```bash
export HUSKY_URDF_EXTRAS=$HOME/Developer/sep799-sidewalk/catkin_ws/src/husky_oakd_stereo_nav/urdf/mcm07_customization.urdf.xacro
roslaunch husky_viz view_robot.launch
```

If using a real OAK-D device, launch Terminal 4:
```bash
roslaunch depthai_examples stereo_inertial_node.launch \
    lrcheck:=true \
    extended:=false\
    subpixel:=true \
    rectify:=true \
    depth_aligned:=true \
    enableSpatialDetection:=false \
    syncNN:=false \
    monoResolution:="800p" \
    rgbResolution:="1080p" \
    confidence:=245 \
    LRchecktresh:=10
```

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