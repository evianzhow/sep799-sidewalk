# Husky + OAK-D Navigation Simulation using dockerized rtabmap_ros

Terminal 1:
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
roslaunch husky_oak_navigation demo_husky_oakd_navigation.launch
```

Terminal 2:
```bash
export HUSKY_URDF_EXTRAS=$HOME/Developer/sep799-sidewalk/catkin_ws/src/husky_oak_navigation/urdf/oak-d.urdf.xacro
roslaunch husky_viz view_robot.launch
```

Terminal 3:
```bash
export HUSKY_URDF_EXTRAS=$HOME/Developer/sep799-sidewalk/catkin_ws/src/husky_oak_navigation/urdf/oak-d.urdf.xacro
roslaunch husky_gazebo husky_playpen.launch
```