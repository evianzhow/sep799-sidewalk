# EuRoC Dataset Testing using RTAB-Map

https://github.com/introlab/rtabmap_ros/blob/noetic-devel/rtabmap_examples/launch/euroc_datasets.launch

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
   -e LD_LIBRARY_PATH=/opt/ros/noetic/lib:/opt/ros/noetic/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu/tegra \
    introlab3it/rtabmap_ros:noetic-latest \
    /bin/bash -c "roslaunch rtabmap_examples euroc_datasets.launch"
```

Terminal 2:
```bash
rosbag play --clock V1_01_easy.bag
```