# OAK-D Handheld Mapping Test

Terminal 1:

```bash
roslaunch depthai_examples stereo_inertial_node.launch enableRviz:=false
```

Terminal 2:
```bash
rosrun imu_filter_madgwick imu_filter_node \
   imu/data_raw:=/stereo_inertial_publisher/imu \
   imu/data:=/stereo_inertial_publisher/imu/data  \
   _use_mag:=false \
   _publish_tf:=false
```

Terminal 3:
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
    /bin/bash -c "export ROS_NAMESPACE=rtabmap && roslaunch rtabmap_launch rtabmap.launch \
      args:="--delete_db_on_start" \
      rgb_topic:=/stereo_inertial_publisher/color/image \
      depth_topic:=/stereo_inertial_publisher/stereo/depth \
      camera_info_topic:=/stereo_inertial_publisher/color/camera_info \
      imu_topic:=/stereo_inertial_publisher/imu/data \
      frame_id:=oak-d_frame \
      approx_sync:=true \
      wait_imu_to_init:=true"
```



