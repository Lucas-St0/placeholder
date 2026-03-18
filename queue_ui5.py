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
from std_srvs.srv import Empty as EmptySrv

class GreenhouseManager:
    def __init__(self, greenhouse_name):
        rospy.init_node('greenhouse_manager')
        
        self.filename = f"/home/lucas/catkin_ws/src/greenhouse_project/config/{greenhouse_name}.json"
        
        # Setting variables
        self.SUCCESS_DISTANCE_THRESHOLD = 0.2
        self.MAX_TOTAL_TIME = 300.0
        self.WATER_THRESHOLD = 40.0 
        self.HEALTHY_MAX = 59.0     
        self.WEIGHTING = 10.0

        self.current_pose = None
        self.priority_override_id = None 
        
        # Publisher and Subscribers
        self.ui_moisture_pub = rospy.Publisher('/plant/moisture_alert', String, queue_size=10)
        self.ui_state_pub = rospy.Publisher('/ridgeback/state', Int32, queue_size=10)
        self.ui_feedback_pub = rospy.Publisher('/dispense_water/feedback', String, queue_size=10)
        self.ui_plan_pub = rospy.Publisher('/plantcare/ordered_queue', String, queue_size=10)
        self.marker_pub = rospy.Publisher('/plant_status_markers', MarkerArray, queue_size=10, latch=True)

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/plantcare/priority_override', Int32, self.priority_callback)

        # Navigation using move_base
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
        if not self.current_pose: return float('inf')

        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose = self.current_pose

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x, goal.pose.position.y = target_x, target_y
        goal.pose.orientation.w = 1.0

        # Calculate straight-line distance as a baseline
        euc_dist = math.sqrt((target_x - start.pose.position.x)**2 + 
                             (target_y - start.pose.position.y)**2)

        try:
            # Request plan from current pose to plant pose
            plan = self.planner_service(start, goal, 2.0)
            
            if not plan.plan.poses:
                # FALLBACK: If planner is blocked by an obstacle, return 
                # straight-line distance with a 50% penalty.
                return euc_dist * 1.5
            
            dist = 0.0
            poses = plan.plan.poses
            for i in range(len(poses) - 1):
                p1 = poses[i].pose.position
                p2 = poses[i+1].pose.position
                dist += math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)

            # Convert moisture to decimal
            moisture_decimal = p['moisture'] / 100.0
            
            # NEW FORMULA
            score = ((1.0 - moisture_decimal) * self.WEIGHTING) / (dist + 1.0)
            
            return dist if dist > 0 else euc_dist
        except:
            # fallback
            return euc_dist * 1.5

    def publish_mission_plan(self):
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
        current_time = time.time()

        for p_id, p in self.plants.items():
            if not p['needs_water'] or p['failure_count'] >= 2 : continue

            last_fail = p.get('last_fail_time', 0)
            if (current_time - last_fail) < 60.0:
                continue
            
            real_dist = self.get_path_distance(p['x'], p['y'])
            moisture_decimal = p['moisture'] / 100.0
            score = ((1.0 - moisture_decimal) * self.WEIGHTING) / (real_dist + 1.0)
            
            if score > best_score:
                best_score, best_plant = score, p_id
        return best_plant

    def water_sequence(self, target_name):
        print(f"--- Watering {target_name} ---")
        p_id = int(target_name.split('_')[1])
        
        self.ui_state_pub.publish(3) # UI State: Dispensing
        for i in range(0, 101, 25):
            self.ui_feedback_pub.publish(f"{i},Pumping")
            time.sleep(0.4)
        
        # After watering, set to 60% (Healthy Green)
        self.plants[target_name]['moisture'] = 60.0
        self.plants[target_name]['needs_water'] = False
        self.ui_moisture_pub.publish(f"{p_id},50.0")
        print(f"--- {target_name} Healthy ---")
    
    def init_plant_states(self):

        thirsty_count = 0

        for p_name in self.plants:
            moisture = random.gauss(42.0, 8.0)
            moisture = max(15.0, min(self.HEALTHY_MAX, moisture))

            needs_water = moisture < self.WATER_THRESHOLD
            if needs_water: thirsty_count += 1
            
            self.plants[p_name].update({
                'moisture': moisture,
                'needs_water': needs_water,
                'failure_count': 0
            })
        print(f"--- Distribution: {thirsty_count}/{len(self.plants)} plants (approx 40%) are Underwatered ---")
        print(f"--- Distribution: 0 plants are Overwatered ---")

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
        # Wait for Localisation
        while not rospy.is_shutdown() and self.current_pose is None:
            rospy.loginfo_once("Waiting for AMCL Pose (Robot Location)...")
            rospy.sleep(1.0)

        rospy.sleep(2.0) # Wait for RViz to load

        # Show all plant states
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

        # Main loop
        while not rospy.is_shutdown():
            self.ui_state_pub.publish(0) # State: Available
            self.publish_mission_plan()  # Calculate queue based on Global Path

            target = self.get_next_target()
            if not target:
                self.ui_plan_pub.publish("") # Clear the UI list
                rospy.loginfo("--- MISSION COMPLETE: All plants are healthy ---")
                break

            # Start navigation
            rospy.loginfo(f"Next Target: {target}")
            self.ui_state_pub.publish(1) # State: Moving
            
            data = self.plants[target]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = data['x']
            goal.target_pose.pose.position.y = data['y']
            goal.target_pose.pose.orientation.w = 1.0

            self.client.send_goal(goal)
            start_t = time.time()
            
            while not rospy.is_shutdown():
                # Check for Manual User Interrupt from the UI
                if self.priority_override_id:
                    rospy.logwarn("Manual Priority received. Diverting robot...")
                    self.client.cancel_goal()
                    break

                state = self.client.get_state()

                dist_to_goal = math.sqrt(
                    (data['x'] - self.current_pose.position.x)**2 + 
                    (data['y'] - self.current_pose.position.y)**2
                )
                
                # Check if Goal Reached (3 = SUCCEEDED)
                if state == 3 or dist_to_goal < self.SUCCESS_DISTANCE_THRESHOLD:
                    rospy.loginfo(f"Reached {target}. Starting watering sequence...")
                    
                    # Watering on UI
                    self.ui_state_pub.publish(3) # State: Dispensing
                    
                    # Simulate the watering time/feedback
                    for i in range(0, 101, 25):
                        self.ui_feedback_pub.publish(f"{i},Watering {target}")
                        time.sleep(0.5)

                    # Update plant node
                    self.plants[target]['moisture'] = 60.0
                    self.plants[target]['needs_water'] = False
                    
                    # Send the update to the Visualizer Topic
                    p_id = int(target.split('_')[1])
                    self.ui_moisture_pub.publish(f"{p_id},85.0")
                    
                    # Update RViz 3D markers and the Job Queue
                    self.publish_rviz_markers()
                    self.publish_mission_plan()
                    
                    rospy.loginfo(f"Successfully watered {target}. Marker updated to Green.")
                    break
                
                # Failure check or timeout
                state = self.client.get_state()
                
                # If navigation fails (4=Aborted, 5=Rejected, 9=Lost) or Timeout
                if state in [4, 5, 9] or (time.time() - start_t > self.MAX_TOTAL_TIME):
                    self.client.cancel_goal()
                    
                    # Increment the failure count for this plant
                    self.plants[target]['failure_count'] += 1
                    
                    current_fails = self.plants[target]['failure_count']
                    
                    if current_fails == 1:
                        rospy.logwarn(f"RETRY LOGIC: Failed to reach {target}. Skipping for now.")
                        self.plants[target]['last_fail_time'] = time.time()
                    
                    elif current_fails >= 2:
                        # DISPLAY ERROR MESSAGE
                        rospy.logerr("************************************************")
                        rospy.logerr(f"ERROR: {target} UNREACHABLE AFTER 2 ATTEMPTS.")
                        rospy.logerr("Please check for physical obstacles or map errors.")
                        rospy.logerr("************************************************")
                    
                    break # Exit the monitoring loop to pick a different target

                rospy.sleep(0.2)

                
if __name__ == '__main__':
    manager = GreenhouseManager("greenhouse2")
    manager.run_mission()