#!/usr/bin/env python3
import rospy
import actionlib
import json
import math
import random
import time
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Quaternion, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler

class GreenhouseManager:
    def __init__(self, greenhouse_name):
        rospy.init_node('greenhouse_manager')
        self.filename = f"/home/lucas/catkin_ws/src/greenhouse_project/config/{greenhouse_name}.json"
        
        # --- Constants ---
        self.TANK_CAPACITY = 10.0
        self.WATER_PER_PLANT = 0.5
        self.FAILURE_COOLDOWN = 120.0       
        self.SUCCESS_DISTANCE_THRESHOLD = 0.25 
        self.MAX_DEVIATION = 0.40           
        self.DRIFT_TIMEOUT = 60.0
        
        # --- State ---
        self.current_water = self.TANK_CAPACITY
        self.current_pose = None
        self.global_path = None
        self.last_scan = None 

        # --- ROS Setup ---
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback) 
        
        # Services
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
        # NOTE: Using the standard move_base service name for planning
        rospy.wait_for_service('/move_base/make_plan')
        self.make_plan_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        
        self.plants = self.load_data()
        self.init_plant_states()

    def scan_callback(self, data):
        self.last_scan = data

    def pose_callback(self, data):
        self.current_pose = data.pose.pose

    def path_callback(self, data):
        self.global_path = data

    def load_data(self):
        try:
            with open(self.filename, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            rospy.logerr(f"File {self.filename} not found!")
            sys.exit(1)

    def init_plant_states(self):
        for p in self.plants:
            moisture = max(0, min(1, random.gauss(0.4, 0.2)))
            self.plants[p].update({
                'moisture': moisture,
                'needs_water': moisture < 0.35,
                'failure_count': 0,
                'last_failed_time': 0
            })

    def get_true_path_distance(self, target_x, target_y):
        """Fixed GetPlan Service Call"""
        if not self.current_pose: return 999.9
        
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose = self.current_pose
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = target_x
        goal.pose.position.y = target_y
        goal.pose.orientation.w = 1.0

        try:
            # In rospy, you pass arguments directly to the service proxy call
            resp = self.make_plan_service(start, goal, 0.5)
            path = resp.plan.poses
            if not path or len(path) < 2: return 999.9
            
            dist = 0.0
            for i in range(len(path)-1):
                p1 = path[i].pose.position
                p2 = path[i+1].pose.position
                dist += math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
            return dist
        except Exception as e:
            rospy.logwarn(f"Plan calculation failed: {e}")
            return 999.9

    def calculate_utility(self, plant_id):
        p = self.plants[plant_id]
        if not p['needs_water'] or p['failure_count'] >= 2: return -1
        
        time_since_fail = time.time() - p['last_failed_time']
        if p['failure_count'] == 1 and time_since_fail < self.FAILURE_COOLDOWN: return 0 
        
        true_dist = self.get_true_path_distance(p['x'], p['y'])
        return ((1.0 - p['moisture']) * 10) / (true_dist + 1.0)

    def get_next_target(self):
        best_score, best_plant = -1, None
        for p_id in self.plants:
            score = self.calculate_utility(p_id)
            if score > best_score:
                best_score, best_plant = score, p_id
        return best_plant

    def clear_bench_strafe(self):
        """Lidar-based obstacle avoidance after watering"""
        if not self.last_scan:
            return

        # Split scan into Left/Right arcs
        # Ridgeback laser indices: adjust based on your specific scan (e.g., 0-720)
        mid = len(self.last_scan.ranges) // 2
        quarter = len(self.last_scan.ranges) // 4
        
        left_arc = self.last_scan.ranges[mid : mid + quarter]
        right_arc = self.last_scan.ranges[mid - quarter : mid]

        left_dist = min([r for r in left_arc if r > 0.1] + [5.0])
        right_dist = min([r for r in right_arc if r > 0.1] + [5.0])

        strafe_cmd = Twist()
        if left_dist < right_dist:
            rospy.loginfo(f"Clearing Bench: Strafing RIGHT (Bench detected {left_dist:.2f}m left)")
            strafe_cmd.linear.y = -0.25 
        else:
            rospy.loginfo(f"Clearing Bench: Strafing LEFT (Bench detected {right_dist:.2f}m right)")
            strafe_cmd.linear.y = 0.25  

        start = time.time()
        while (time.time() - start) < 1.2:
            self.cmd_pub.publish(strafe_cmd)
            rospy.sleep(0.1)
        
        self.cmd_pub.publish(Twist()) 
        rospy.sleep(0.5)

    def run_mission(self):
        while not rospy.is_shutdown():
            if self.current_water < self.WATER_PER_PLANT: 
                rospy.logwarn("Tank empty. Refill required.")
                break
                
            target = self.get_next_target()
            if not target: 
                rospy.loginfo("Mission Complete.")
                break

            data = self.plants[target]
            q = Quaternion(*quaternion_from_euler(0, 0, data.get('yaw', 0.0)))
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = data['x']
            goal.target_pose.pose.position.y = data['y']
            goal.target_pose.pose.orientation = q

            self.clear_costmaps()
            self.client.send_goal(goal)
            
            start_time = time.time()
            drift_start = None
            
            while not rospy.is_shutdown():
                state = self.client.get_state()
                error = math.sqrt((data['x']-self.current_pose.position.x)**2 + (data['y']-self.current_pose.position.y)**2)
                
                if state == 3 or error <= self.SUCCESS_DISTANCE_THRESHOLD:
                    self.client.cancel_goal()
                    rospy.loginfo(f"Watered {target}. Strafing away...")
                    self.current_water -= self.WATER_PER_PLANT
                    self.plants[target]['needs_water'] = False
                    self.clear_bench_strafe()
                    break
                    
                if state == 4:
                    rospy.logwarn(f"Target {target} aborted by ROS.")
                    self.plants[target]['failure_count'] += 1
                    self.plants[target]['last_failed_time'] = time.time()
                    break
                    
                rospy.sleep(0.5)

if __name__ == '__main__':
    manager = GreenhouseManager("greenhouse2")
    manager.run_mission()
