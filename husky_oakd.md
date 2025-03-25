# Husky + OAK-D Navigation Stack using dockerized rtabmap_ros

The sequence is important! Launch Gazebo first then rtabmap_ros

Terminal 1:
```bash
export HUSKY_URDF_EXTRAS=$HOME/Developer/sep799-sidewalk/catkin_ws/src/husky_oak_navigation/urdf/oak-d.urdf.xacro
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
roslaunch husky_oak_navigation demo_husky_oakd_navigation.launch
```

If you are using a real device, launch husky_oak_navigation with following command:
```bash
roslaunch husky_oak_navigation demo_husky_oakd_navigation.launch oak_d_type:=device
```

`imu_filter_madgwick` should be automatically launched as a node.

Terminal 3:
```bash
export HUSKY_URDF_EXTRAS=$HOME/Developer/sep799-sidewalk/catkin_ws/src/husky_oak_navigation/urdf/oak-d.urdf.xacro
roslaunch husky_viz view_robot.launch
```

Terminal 4:
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