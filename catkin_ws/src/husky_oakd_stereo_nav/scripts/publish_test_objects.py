#!/usr/bin/env python

import rospy
import math
from vision_msgs.msg import BoundingBox3D, BoundingBox3DArray
from geometry_msgs.msg import Pose, Vector3, Quaternion
from std_msgs.msg import Header

def publish_test_objects():
    # Initialize ROS node
    rospy.init_node('test_objects_publisher', anonymous=True)
    
    # Publisher for /objects_3d topic
    pub = rospy.Publisher('/objects_3d', BoundingBox3DArray, queue_size=10)
    
    # Set publish rate (e.g., 1 Hz)
    rate = rospy.Rate(1)
    
    # Define frame ID
    frame_id = rospy.get_param('~frame_id', 'base_link')
    
    while not rospy.is_shutdown():
        # Create BoundingBox3DArray message
        bbox_array = BoundingBox3DArray()
        bbox_array.header = Header()
        bbox_array.header.stamp = rospy.Time.now()
        bbox_array.header.frame_id = frame_id
        
        # Create a single BoundingBox3D
        bbox = BoundingBox3D()
        
        # Set center pose (position and orientation)
        bbox.center = Pose()
        bbox.center.position.x = 2.0  # XC
        bbox.center.position.y = 1.0  # y
        bbox.center.position.z = 0.5  # z
        
        # Convert yaw to quaternion (rotation around z-axis)
        yaw = 0.785398  # 45 degrees in radians
        bbox.center.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw/2.0),
            w=math.cos(yaw/2.0)
        )
        
        # Set size (length, width, height)
        bbox.size = Vector3()
        bbox.size.x = 1.0  # length
        bbox.size.y = 0.5  # width
        bbox.size.z = 1.0  # height
        
        # Add BoundingBox3D to the array
        bbox_array.boxes.append(bbox)
        
        # Publish the message
        pub.publish(bbox_array)
        rospy.loginfo("Published test object array to /objects_3d")
        
        # Sleep to maintain publish rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_test_objects()
    except rospy.ROSInterruptException:
        pass