#!/usr/bin/env python3
import rospy
import actionlib
import json
import random
import time
import math
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Quaternion
from nav_msgs.msg import Path
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler

class GreenhouseManager:
    def __init__(self, greenhouse_name):
        rospy.init_node('greenhouse_manager')
        self.filename = f"/home/lucas/catkin_ws/src/greenhouse_project/config/{greenhouse_name}.json"
        
        # --- Physical & Logic Constraints ---
        self.TANK_CAPACITY = 10.0
        self.WATER_PER_PLANT = 0.5
        self.FAILURE_COOLDOWN = 120.0       # Cooldown after Strike 1
        self.SUCCESS_DISTANCE_THRESHOLD = 0.25 
        self.MAX_DEVIATION = 0.40           
        self.DRIFT_TIMEOUT = 60.0
        self.MAX_TOTAL_TIME = 300.0         # Safety timeout per goal

        # --- State Variables ---
        self.current_water = self.TANK_CAPACITY
        self.current_pose = None
        self.global_path = None
        self.test_results = []

        # --- Action & Publishers ---
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # --- Subscribers & Services ---
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
        # --- Data Initialization ---
        self.plants = self.load_data()
        self.init_plant_states()

    def load_data(self):
        try:
            with open(self.filename, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            rospy.logerr(f"File {self.filename} not found!")
            sys.exit(1)

    def init_plant_states(self):
        """ Applies Gaussian distribution to moisture and sets initial failure states """
        for p in self.plants:
            # Gaussian: Mean 0.4, StdDev 0.2 (0.0 to 1.0 range)
            moisture = max(0, min(1, random.gauss(0.4, 0.2)))
            self.plants[p].update({
                'moisture': moisture,
                'needs_water': moisture < 0.35, # Only water if thirsty
                'failure_count': 0,
                'last_failed_time': 0
            })
        rospy.loginfo("Greenhouse Moisture Map initialized.")

    def pose_callback(self, data):
        self.current_pose = data.pose.pose

    def path_callback(self, data):
        self.global_path = data

    def calculate_utility(self, plant_id):
        """ The Decision Engine: Weights distance vs moisture urgency """
        p = self.plants[plant_id]
        
        # Rule: Skip if not thirsty or blacklisted (2 strikes)
        if not p['needs_water'] or p['failure_count'] >= 2:
            return -1
        
        # Rule: Apply cooldown after Strike 1
        time_since_fail = time.time() - p['last_failed_time']
        if p['failure_count'] == 1 and time_since_fail < self.FAILURE_COOLDOWN:
            return 0 
        
        if not self.current_pose: return 0
        
        dist = math.sqrt((p['x'] - self.current_pose.position.x)**2 + 
                         (p['y'] - self.current_pose.position.y)**2)
        urgency = 1.0 - p['moisture']
        
        # Utility Score = (Urgency) / (Distance)
        return (urgency * 10) / (dist + 1.0)

    def get_next_target(self):
        best_score = -1
        best_plant = None
        for p_id in self.plants:
            score = self.calculate_utility(p_id)
            if score > best_score:
                best_score = score
                best_plant = p_id
        return best_plant

    def get_path_deviation(self):
        if not self.global_path or not self.current_pose: return 0.0
        min_dist = 999.9
        for p in self.global_path.poses:
            dx = p.pose.position.x - self.current_pose.position.x
            dy = p.pose.position.y - self.current_pose.position.y
            dist = math.sqrt(dx**2 + dy**2)
            if dist < min_dist: min_dist = dist
        return min_dist

    def water_plant(self, plant_id, elapsed_time, error):
        rospy.loginfo(f"SUCCESS: Watered {plant_id} (Tank: {self.current_water:.1f}L)")
        self.plants[plant_id]['moisture'] = 1.0
        self.plants[plant_id]['needs_water'] = False
        self.plants[plant_id]['failure_count'] = 0 
        self.current_water -= self.WATER_PER_PLANT
        self.test_results.append({"id": plant_id, "status": "SUCCESS", "error": error, "time": elapsed_time})

    def handle_failure(self, plant_id, reason, error, elapsed):
        self.plants[plant_id]['failure_count'] += 1
        self.plants[plant_id]['last_failed_time'] = time.time()
        count = self.plants[plant_id]['failure_count']
        
        if count >= 2:
            rospy.logerr(f"CRITICAL: {plant_id} HIT 2 STRIKES ({reason}). BLACKLISTED.")
            self.test_results.append({"id": plant_id, "status": "FAILED_BLACKLIST", "error": error, "time": elapsed})
        else:
            rospy.logwarn(f"STRIKE 1: {plant_id} failed ({reason}). Re-queueing after cooldown.")

    def run_mission(self):
        while not rospy.is_shutdown():
            if self.current_water < self.WATER_PER_PLANT:
                rospy.logwarn("Tank Empty! Mission ending.")
                break

            target = self.get_next_target()
            if not target:
                rospy.loginfo("No more viable targets. Saving data...")
                break

            # Execute Navigation
            data = self.plants[target]
            rospy.loginfo(f"TARGETING: {target} (Moisture: {data['moisture']:.2f})")
            
            q = Quaternion(*quaternion_from_euler(0, 0, data.get('yaw', 0.0)))
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = data['x']
            goal.target_pose.pose.position.y = data['y']
            goal.target_pose.pose.orientation = q

            try: self.clear_costmaps()
            except: pass

            self.client.send_goal(goal)
            start_time = time.time()
            drift_start_time = None
            
            while not rospy.is_shutdown():
                state = self.client.get_state()
                dx = data['x'] - self.current_pose.position.x
                dy = data['y'] - self.current_pose.position.y
                error = math.sqrt(dx**2 + dy**2)
                deviation = self.get_path_deviation()
                elapsed = time.time() - start_time

                # 1. SUCCESS
                if state == 3 or error < self.SUCCESS_DISTANCE_THRESHOLD:
                    self.client.cancel_goal()
                    self.water_plant(target, elapsed, error)
                    break

                # 2. ABORTED
                if state == 4:
                    self.handle_failure(target, "ROS Aborted", error, elapsed)
                    break

                # 3. DRIFT
                if deviation > self.MAX_DEVIATION:
                    if drift_start_time is None: drift_start_time = time.time()
                    if (time.time() - drift_start_time) > self.DRIFT_TIMEOUT:
                        self.client.cancel_goal()
                        self.handle_failure(target, "Drift Timeout", error, elapsed)
                        break
                else:
                    drift_start_time = None

                # 4. MASTER TIMEOUT
                if elapsed > self.MAX_TOTAL_TIME:
                    self.client.cancel_goal()
                    self.handle_failure(target, "Critical Timeout", error, elapsed)
                    break

                rospy.sleep(0.5)
            
            rospy.sleep(1.0) # Short dwell for costmap clearing

        # Final Save
        with open("/home/lucas/catkin_ws/src/greenhouse_project/results_priority.json", 'w') as f:
            json.dump(self.test_results, f, indent=4)

if __name__ == '__main__':
    name = sys.argv[1] if len(sys.argv) > 1 else "greenhouse2"
    manager = GreenhouseManager(name)
    manager.run_mission()
