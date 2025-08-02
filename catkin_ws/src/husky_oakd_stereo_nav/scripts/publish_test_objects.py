#!/usr/bin/env python

import rospy
import math
import random
from vision_msgs.msg import BoundingBox3D, BoundingBox3DArray
from geometry_msgs.msg import Pose, Vector3, Quaternion
from std_msgs.msg import Header

def publish_test_objects():
    # Initialize ROS node
    rospy.init_node('test_objects_publisher', anonymous=True)
    
    # Publisher for /objects_3d topic
    pub = rospy.Publisher('/objects_3d', BoundingBox3DArray, queue_size=10)
    
    # Set publish rate (1 Hz)
    rate = rospy.Rate(1)
    
    # Define frame ID
    frame_id = rospy.get_param('~frame_id', 'base_link')
    
    # Initialize object state
    position = [2.0, 1.0, 0.85]  # [x, y, z] (z is half height for ground contact)
    yaw = 0.0  # Initial yaw in radians
    speed = random.uniform(0.01, 0.1)  # Random speed between 0.01 and 0.1 m/s
    direction = random.uniform(0, 2 * math.pi)  # Random initial direction (radians)
    
    # Human-sized box dimensions
    box_length = 0.5  # shoulder width in meters
    box_width = 0.3   # body depth in meters
    box_height = 1.7  # average human height in meters
    
    while not rospy.is_shutdown():
        # Update position based on speed and direction
        dt = 1.0  # Time step (1 second at 1 Hz)
        position[0] += speed * math.cos(direction) * dt  # Update x
        position[1] += speed * math.sin(direction) * dt  # Update y
        # z remains constant as human stays on ground
        
        # Occasionally change direction (mimic turning)
        if random.random() < 0.3:  # 30% chance to change direction each cycle
            direction = random.uniform(0, 2 * math.pi)  # New random direction
        
        # Occasionally change yaw (mimic human turning)
        if random.random() < 0.3:  # 30% chance to change yaw each cycle
            yaw = random.uniform(0, 2 * math.pi)  # Random yaw
        
        # Create BoundingBox3DArray message
        bbox_array = BoundingBox3DArray()
        bbox_array.header = Header()
        bbox_array.header.stamp = rospy.Time.now()
        bbox_array.header.frame_id = frame_id
        
        # Create a single BoundingBox3D
        bbox = BoundingBox3D()
        
        # Set center pose (position and orientation)
        bbox.center = Pose()
        bbox.center.position.x = position[0]
        bbox.center.position.y = position[1]
        bbox.center.position.z = position[2]
        
        # Convert yaw to quaternion (rotation around z-axis)
        bbox.center.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw/2.0),
            w=math.cos(yaw/2.0)
        )
        
        # Set size (human-sized box)
        bbox.size = Vector3()
        bbox.size.x = box_length  # length
        bbox.size.y = box_width   # width
        bbox.size.z = box_height  # height
        
        # Add BoundingBox3D to the array
        bbox_array.boxes.append(bbox)
        
        # Publish the message
        pub.publish(bbox_array)
        rospy.loginfo(f"Published test object array to /objects_3d at position ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}), yaw {yaw:.2f}")
        
        # Sleep to maintain publish rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_test_objects()
    except rospy.ROSInterruptException:
        pass