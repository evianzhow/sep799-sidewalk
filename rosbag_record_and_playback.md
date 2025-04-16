### Recording

On the husky robot:

Terminal 1:

```bash
roslaunch depthai_examples stereo_inertial_node.launch enableRviz:=false
```

rtabmap requires following data: 
- /stereo_inertial_publisher/color/image
- /stereo_inertial_publisher/stereo/depth
- /stereo_inertial_publisher/color/camera_info
- /stereo_inertial_publisher/imu

We will use `/stereo_inertial_publisher/color/image/compressed` and `/stereo_inertial_publisher/stereo/depth/compressedDepth` to eliminate the rosbag size:

Terminal 2:

```bash
rosbag record /stereo_inertial_publisher/color/image/compressed \
              /stereo_inertial_publisher/stereo/depth/compressedDepth \
              /stereo_inertial_publisher/color/camera_info \
              /stereo_inertial_publisher/imu
```

### Playback

On remote Jetson machine:

Terminal 1:

```bash
cd sep799-sidewalk/
roslaunch ros_bag_playback.launch
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

Terminal 3:

```bash
rosbag play --clock 0416_marc_corridor.bag
```

### Hint: Unlock Lock Screen from TigerVNC

```bash
loginctl unlock-sessions
```
