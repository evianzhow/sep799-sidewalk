#!/usr/bin/env python

import rospy
import os
import subprocess
from nav_msgs.msg import OccupancyGrid
from datetime import datetime

class MapSaver:
    def __init__(self):
        rospy.init_node('map_saver_node', anonymous=True)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.map_received = False
        self.last_map_time = 0.0
        self.save_interval = 1.0  # Save every 1 second
        self.output_dir = rospy.get_param('~output_dir', '/tmp/maps')
        
        # Create output directory if it doesn't exist
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        rospy.loginfo(f"Map saver initialized. Saving maps to {self.output_dir}")

    def map_callback(self, msg):
        self.map_received = True
        current_time = rospy.get_time()
        
        # Check if enough time has passed since last save
        if current_time - self.last_map_time >= self.save_interval:
            self.save_map()
            self.last_map_time = current_time

    def save_map(self):
        try:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            map_name = f"map_{timestamp}"
            map_path = os.path.join(self.output_dir, map_name)
            
            # Call map_saver command
            cmd = ['rosrun', 'map_server', 'map_saver', '-f', map_path]
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                rospy.loginfo(f"Saved map: {map_path}.pgm and {map_path}.yaml")
            else:
                rospy.logerr(f"Failed to save map: {result.stderr}")
                
        except Exception as e:
            rospy.logerr(f"Error saving map: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        saver = MapSaver()
        saver.run()
    except rospy.ROSInterruptException:
        pass