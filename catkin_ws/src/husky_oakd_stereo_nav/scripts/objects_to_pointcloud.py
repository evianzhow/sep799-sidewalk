#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import Point32, Point
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox3DArray, BoundingBox3D
from visualization_msgs.msg import Marker, MarkerArray

class ObjectToCostmap:
    def __init__(self):
        rospy.init_node('object_to_costmap', anonymous=True)
        
        # Parameters
        self.frame_id = rospy.get_param('~frame_id', 'base_link')  # Coordinate frame (LiDAR frame)
        self.obstacle_topic = rospy.get_param('~obstacle_topic', '/obstacle_cloud')  # Topic for PointCloud
        self.object_topic = rospy.get_param('~object_topic', '/objects_3d')  # Input 3D objects topic
        self.point_density = rospy.get_param('~point_density', 10000.0)  # Points per cubic meter
        
        # Publishers
        self.obstacle_pub = rospy.Publisher(self.obstacle_topic, PointCloud, queue_size=10)
        
        # Subscribers
        self.object_sub = rospy.Subscriber(self.object_topic, BoundingBox3DArray, self.object_callback)
        
        # TF listener for coordinate transformations
        self.tf_listener = tf.TransformListener()

    def object_callback(self, msg):
        point_cloud = PointCloud()
        point_cloud.header = Header()
        point_cloud.header.stamp = rospy.Time.now()
        point_cloud.header.frame_id = self.frame_id
        
        for box in msg.boxes:
            # Extract 7-parameter format [XC, y, z, l, w, h, yaw]
            xc = box.center.position.x
            yc = box.center.position.y
            zc = box.center.position.z
            l = box.size.x  # Length (X-axis)
            w = box.size.y  # Width (Y-axis)
            h = box.size.z  # Height (Z-axis)
            # Convert quaternion to yaw
            yaw = tf.transformations.euler_from_quaternion([
                box.center.orientation.x,
                box.center.orientation.y,
                box.center.orientation.z,
                box.center.orientation.w
            ])[2]  # Yaw (rotation around Z-axis)

            # Generate cubic point cloud for the bounding box
            points = self.create_cubic_point_cloud(xc, yc, zc, l, w, h, yaw)
            point_cloud.points.extend(points)
        
        # Publish point cloud
        self.obstacle_pub.publish(point_cloud)

    def create_cubic_point_cloud(self, xc, yc, zc, l, w, h, yaw):
        """Generate a cubic point cloud enveloping the 3D bounding box."""
        points = []
        
        # Calculate number of points based on volume and density
        volume = l * w * h
        num_points = int(self.point_density * volume)
        if num_points < 8:  # Minimum 8 points (cube corners)
            num_points = 8
        
        # Sample points on the surface of the bounding box
        # We'll place points on the 6 faces of the cube
        half_l = l / 2.0
        half_w = w / 2.0
        half_z = h / 2.0
        
        # Define steps for sampling points on each face
        step = (l / math.pow(num_points / 6, 1/2)) if num_points > 6 else l / 2.0
        if step == 0:
            step = 0.1  # Avoid division by zero
        
        # Generate points on each face
        for face in ['top', 'bottom', 'front', 'back', 'left', 'right']:
            if face in ['top', 'bottom']:
                z = half_z if face == 'top' else -half_z
                for x in self.frange(-half_l, half_l, step):
                    for y in self.frange(-half_w, half_w, step):
                        points.append(self.transform_point(x, y, z, xc, yc, zc, yaw))
            elif face in ['front', 'back']:
                x = half_l if face == 'front' else -half_l
                for y in self.frange(-half_w, half_w, step):
                    for z in self.frange(-half_z, half_z, step):
                        points.append(self.transform_point(x, y, z, xc, yc, zc, yaw))
            elif face in ['left', 'right']:
                y = half_w if face == 'right' else -half_w
                for x in self.frange(-half_l, half_l, step):
                    for z in self.frange(-half_z, half_z, step):
                        points.append(self.transform_point(x, y, z, xc, yc, zc, yaw))
        
        return points

    def transform_point(self, x, y, z, xc, yc, zc, yaw):
        """Transform a point from local box frame to global frame."""
        # Rotate by yaw
        x_rot = x * math.cos(yaw) - y * math.sin(yaw)
        y_rot = x * math.sin(yaw) + y * math.cos(yaw)
        z_rot = z  # No rotation around X or Y
        
        # Translate to global frame
        x_global = xc + x_rot
        y_global = yc + y_rot
        z_global = zc + z_rot
        
        point = Point()
        point.x = x_global
        point.y = y_global
        point.z = z_global
        return point

    def frange(self, start, stop, step):
        """Helper function to generate float range."""
        while start <= stop:
            yield start
            start += step

    def create_visualization_marker(self, xc, yc, l, w, yaw, marker_id):
        """Create a visualization marker for RViz (2D footprint)."""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacles"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Position (center of the object)
        marker.pose.position.x = xc
        marker.pose.position.y = yc
        marker.pose.position.z = 0.0  # Ground plane for 2D visualization
        
        # Orientation (yaw)
        marker.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
        
        # Scale (l, w, small height for 2D visualization)
        marker.scale.x = l
        marker.scale.y = w
        marker.scale.z = 0.1  # Thin for visualization
        
        # Color (red, semi-transparent)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        
        marker.lifetime = rospy.Duration(0.5)  # Short lifetime to refresh
        return marker

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObjectToCostmap()
        node.run()
    except rospy.ROSInterruptException:
        pass