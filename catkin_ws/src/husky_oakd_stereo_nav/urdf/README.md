## Customized URDF Folder

---

This folder contains customized URDF files for simulating a real-world-like, modified A200 Husky robot integrated with an Ouster LiDAR, sensor arch, SwiftNav GPS, and OAK-D stereo camera.

### Files

* `mcm07_customization.urdf.xacro`: The main URDF file, included by setting the `HUSKY_URDF_EXTRAS` environment variable. By default, it includes `oak-d_kinect.urdf.xacro`.
* `oak-d_kinect.urdf.xacro`: A standalone URDF file for the OAK-D stereo camera with a Gazebo plugin that emulates a Kinect-like RGB-D camera setup (1 RGB and 1 Depth/IR-like camera).
* `oak-d_stereo.urdf.xacro`: A standalone URDF file for the OAK-D stereo camera with a Gazebo plugin that emulates a stereo camera setup (1 RGB and 2 mono cameras).
* `oak-d-urdf.xacro.orig`: An initial version used by `rgbd_vslam_no_visual_odom.launch`. Created during 2025 Winter term. It contains only the OAK-D stereo camera and an arbitrary mounting arch.