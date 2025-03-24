# Turtlebot3 Navigation Simulation using dockerized rtabmap_ros

The sequence is important! Launch Gazebo first then rtabmap_ros

Terminal 1:
```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
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
   -e LD_LIBRARY_PATH=/opt/ros/noetic/lib:/opt/ros/noetic/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu/tegra \
    introlab3it/rtabmap_ros:noetic-latest \
    /bin/bash
```

After entering the bash:
```bash
unset ROS_NAMESPACE
export TURTLEBOT3_MODEL=waffle
export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1:$LD_PRELOAD
roslaunch rtabmap_demos demo_turtlebot3_navigation.launch
```

