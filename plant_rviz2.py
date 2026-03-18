#!/usr/bin/env python3

import rospy
import json
import sys
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

class DynamicVisualizer:
    def __init__(self, greenhouse_name):
        rospy.init_node('plant_dynamic_visualizer')
        
        # 1. Configuration
        self.WATER_THRESHOLD = 40.0
        self.filename = f"/home/lucas/catkin_ws/src/greenhouse_project/config/{greenhouse_name}.json"
        
        # 2. Setup Publisher and Subscriber
        self.marker_pub = rospy.Publisher('/plant_locations', MarkerArray, queue_size=10, latch=True)
        rospy.Subscriber('/plant/moisture_alert', String, self.moisture_callback)

        # 3. Load coordinates from JSON
        self.marker_dict = {} # We store markers here so we can update them by ID
        self.load_initial_positions()
        
        rospy.loginfo(f"Visualizer ready for {greenhouse_name}. Listening for updates...")
        rospy.spin()

    def load_initial_positions(self):
        try:
            with open(self.filename, 'r') as f:
                plants = json.load(f)
            
            for p_name, data in plants.items():
                p_id = int(p_name.split('_')[1])
                
                # Create a permanent marker for this plant
                m = Marker()
                m.header.frame_id = "map"
                m.id = p_id
                m.type = Marker.CYLINDER
                m.pose.position.x = data['x']
                m.pose.position.y = data['y']
                m.pose.position.z = 0.2
                m.scale.x, m.scale.y, m.scale.z = (0.3, 0.3, 0.4)
                
                # Start them all as Green
                m.color.r, m.color.g, m.color.b, m.color.a = (0.0, 1.0, 0.0, 0.8)
                
                self.marker_dict[p_id] = m
                
            self.publish_rviz_markers()
        except Exception as e:
            rospy.logerr(f"Failed to load JSON: {e}")

    def moisture_callback(self, msg):
        """ Triggered whenever '/plant/moisture_alert' sends 'ID,Level' """
        try:
            # Parse "5,35.0" -> ID 5, Moisture 35.0
            p_id_str, level_str = msg.data.split(",")
            p_id = int(p_id_str)
            level = float(level_str)

            if p_id in self.marker_dict:
                marker = self.marker_dict[p_id]
                
                # Update Color based on level
                if level < self.WATER_THRESHOLD:
                    # Change to Orange
                    marker.color.r, marker.color.g, marker.color.b = (1.0, 0.5, 0.0)
                else:
                    # Change to Green
                    marker.color.r, marker.color.g, marker.color.b = (0.0, 1.0, 0.0)
                
                self.publish_rviz_markers()
        except ValueError:
            pass

    def publish_rviz_markers(self):
        """ Publishes the current state of all markers to RViz """
        msg = MarkerArray()
        msg.markers = list(self.marker_dict.values())
        self.marker_pub.publish(msg)

if __name__ == '__main__':
    name = sys.argv[1] if len(sys.argv) > 1 else "greenhouse2"
    DynamicVisualizer(name)