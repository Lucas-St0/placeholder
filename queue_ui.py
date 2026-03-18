#!/usr/bin/env python3

import rospy
import actionlib
import json
import random
import time
import math
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseStamped
from nav_msgs.srv import GetPlan
from std_msgs.msg import String, Int32
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler

class GreenhouseManager:
    def __init__(self, greenhouse_name):
        rospy.init_node('greenhouse_manager')
        
        # 1. PATH CONFIGURATION
        self.filename = f"/home/lucas/catkin_ws/src/greenhouse_project/config/{greenhouse_name}.json"
        
        # 2. MISSION CONSTRAINTS
        self.SUCCESS_DISTANCE_THRESHOLD = 0.25 
        self.MAX_TOTAL_TIME = 300.0
        self.WATER_THRESHOLD = 40.0 
        self.HEALTHY_MAX = 59.0     
        self.WEIGHTING = 10.0 # From your formula logic

        # 3. STATE VARIABLES
        self.current_pose = None
        self.priority_override_id = None 
        
        # 4. ROS PUB/SUB
        self.ui_moisture_pub = rospy.Publisher('/plant/moisture_alert', String, queue_size=10)
        self.ui_state_pub = rospy.Publisher('/ridgeback/state', Int32, queue_size=10)
        self.ui_feedback_pub = rospy.Publisher('/dispense_water/feedback', String, queue_size=10)
        self.ui_plan_pub = rospy.Publisher('/plantcare/ordered_queue', String, queue_size=10)
        self.marker_pub = rospy.Publisher('/plant_status_markers', MarkerArray, queue_size=10, latch=True)
        
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/plantcare/priority_override', Int32, self.priority_callback)

        # 5. NAVIGATION & PLANNING
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # Connect to the Global Planner service
        rospy.loginfo("Waiting for move_base/make_plan service...")
        rospy.wait_for_service('/move_base/make_plan')
        self.planner_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        
        self.plants = self.load_data()
        self.init_plant_states()

        self.marker_dict = {}
        
        print(f"--- Greenhouse Manager Ready (Global Path Logic Active) ---")

    def load_data(self):
        with open(self.filename, 'r') as f:
            return json.load(f)

    def pose_callback(self, data):
        self.current_pose = data.pose.pose

    def priority_callback(self, msg):
        target_key = f"Plant_{msg.data:03d}"
        if target_key in self.plants:
            self.priority_override_id = target_key
            self.client.cancel_all_goals()

    def get_path_distance(self, target_x, target_y):
        """ Asks the Global Planner for the actual distance around benches """
        if not self.current_pose: return float('inf')

        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose = self.current_pose

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = target_x
        goal.pose.position.y = target_y
        goal.pose.orientation.w = 1.0

        try:
            # Request plan from current pose to plant pose (tolerance 0.5m)
            plan = self.planner_service(start, goal, 0.5)
            
            dist = 0.0
            poses = plan.plan.poses
            for i in range(len(poses) - 1):
                p1 = poses[i].pose.position
                p2 = poses[i+1].pose.position
                dist += math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            
            # If planner fails to find a path, return a high number to avoid it
            return dist if dist > 0 else 999.0
        except:
            return 999.0

    def publish_mission_plan(self):
        """ Calculates queue using actual path distance, not straight lines """
        if not self.current_pose: return
        thirsty_list = []

        for p_id, p in self.plants.items():
            if p['needs_water']:
                # Calculate the REAL distance around obstacles
                real_dist = self.get_path_distance(p['x'], p['y'])
                
                # Apply your custom formula
                moisture_decimal = p['moisture'] / 100.0
                score = ((1.0 - moisture_decimal) * self.WEIGHTING) / (real_dist + 1.0)
                
                thirsty_list.append({'id': p_id, 'score': score})
        
        thirsty_list.sort(key=lambda x: x['score'], reverse=True)
        ordered_ids = [str(int(p['id'].split('_')[1])) for p in thirsty_list]
        self.ui_plan_pub.publish(",".join(ordered_ids))

    def get_next_target(self):
        if self.priority_override_id:
            target = self.priority_override_id
            self.priority_override_id = None
            return target

        best_score, best_plant = -1, None
        for p_id, p in self.plants.items():
            if not p['needs_water']: continue
            
            real_dist = self.get_path_distance(p['x'], p['y'])
            moisture_decimal = p['moisture'] / 100.0
            score = ((1.0 - moisture_decimal) * self.WEIGHTING) / (real_dist + 1.0)
            
            if score > best_score:
                best_score, best_plant = score, p_id
        return best_plant

    # ... [Rest of your watering and mission loop logic remains the same] ...
    
    def init_plant_states(self):
        for p_name in self.plants:
            moisture = random.uniform(15.0, 55.0)
            self.plants[p_name].update({'moisture': moisture, 'needs_water': moisture < self.WATER_THRESHOLD})

    def publish_rviz_markers(self):
        """ Publishes the current state of all markers to RViz """
        msg = MarkerArray()
        msg.markers = list(self.marker_dict.values())
        self.marker_pub.publish(msg)    

    def run_mission(self):
        """
        Main mission control loop. 
        Handles initial UI sync, navigation, and moisture updates.
        """
        # 1. Wait for Localisation
        while not rospy.is_shutdown() and self.current_pose is None:
            rospy.loginfo_once("Waiting for AMCL Pose (Robot Location)...")
            rospy.sleep(1.0)

        rospy.sleep(2.0) # Grace period for RViz to load

        # 2. INITIAL SYNC: Tell the Visualizer the starting state of ALL plants
        rospy.loginfo("Syncing initial plant states with RViz...")
        for p_name, data in self.plants.items():
            try:
                p_id = int(p_name.split('_')[1])
                # This triggers the initial Green/Orange colors in your RViz script
                self.ui_moisture_pub.publish(f"{p_id},{data['moisture']:.1f}")
                time.sleep(0.01) # Small delay to prevent dropped ROS messages
            except (IndexError, ValueError):
                continue
        
        self.publish_rviz_markers() # Update the 3D Marker cylinders

        # 3. MAIN MISSION LOOP
        while not rospy.is_shutdown():
            self.ui_state_pub.publish(0) # State: Available
            self.publish_mission_plan()  # Calculate queue based on Global Path

            target = self.get_next_target()
            if not target:
                self.ui_plan_pub.publish("") # Clear the UI list
                rospy.loginfo("--- MISSION COMPLETE: All plants are healthy ---")
                break

            # 4. NAVIGATION START
            rospy.loginfo(f"Next Target: {target}")
            self.ui_state_pub.publish(1) # State: Moving
            
            data = self.plants[target]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = data['x']
            goal.target_pose.pose.position.y = data['y']
            # Simple orientation (facing 'forward' on the map)
            goal.target_pose.pose.orientation.w = 1.0

            self.client.send_goal(goal)
            start_t = time.time()
            
            # 5. MONITOR NAVIGATION
            while not rospy.is_shutdown():
                # Check for Manual User Interrupt from the UI
                if self.priority_override_id:
                    rospy.logwarn("Manual Priority received. Diverting robot...")
                    self.client.cancel_goal()
                    break

                state = self.client.get_state()
                
                # Check if Goal Reached (3 = SUCCEEDED)
                if state == 3:
                    rospy.loginfo(f"Reached {target}. Starting watering sequence...")
                    
                    # 6. WATERING SEQUENCE
                    self.ui_state_pub.publish(3) # State: Dispensing
                    
                    # Simulate the watering time/feedback
                    for i in range(0, 101, 25):
                        self.ui_feedback_pub.publish(f"{i},Watering {target}")
                        time.sleep(0.5)

                    # 7. DATA UPDATE & COLOR TRIGGER
                    # Reset moisture to healthy level
                    self.plants[target]['moisture'] = 85.0
                    self.plants[target]['needs_water'] = False
                    
                    # Send the update to the Visualizer Topic
                    p_id = int(target.split('_')[1])
                    self.ui_moisture_pub.publish(f"{p_id},85.0")
                    
                    # Update RViz 3D markers and the Job Queue
                    self.publish_rviz_markers()
                    self.publish_mission_plan()
                    
                    rospy.loginfo(f"Successfully watered {target}. Marker updated to Green.")
                    break
                
                # 8. TIMEOUT / FAILURE CHECK
                if time.time() - start_t > self.MAX_TOTAL_TIME:
                    rospy.logerr(f"Failed to reach {target} within time limit. Skipping.")
                    self.client.cancel_goal()
                    break
                
                if state in [4, 5, 9]: # Aborted, Rejected, or Lost
                    rospy.logerr(f"Navigation to {target} failed (State: {state})")
                    break

                rospy.sleep(0.2)

                
if __name__ == '__main__':
    manager = GreenhouseManager("greenhouse2")
    manager.run_mission()
