#!/usr/bin/env python3

# Importing the required libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
import random
from queue import PriorityQueue
import cv2

# Function for converting the quaternion to euler angles
def quaternion_to_euler(x,y,z,w):
    theta0 = +2.0 * (w * x + y * z)
    theta1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(theta0,theta1)

    theta2 = +2.0 * (w * y - z * x)
    theta2 = +1.0 if theta2 > +1.0 else theta2
    theta2 = -1.0 if theta2 < -1.0 else theta2
    pitch_y = math.asin(theta2)
    
    theta3 = +2.0 * (w * z + x * y)
    theta4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(theta3, theta4)
    
    return roll_x, pitch_y, yaw_z 

# Canvas dimensions
canvas_height = 200
canvas_width = 600

# Define the colors
clearance_color = (127, 127, 127)
obstacle_color = (0, 0, 0)
free_space_color = (255, 255, 255)
threshold = 2
path_color = (0, 255, 0)
clearance_distance = 5
robo_radius = 22
nodes = []
obs =set()

# Initializing a white canvas
canvas = np.ones((canvas_height, canvas_width, 3), dtype="uint8") * 255

# Defining obstacles using half plane model
def obstacles(node):
    x, y = node
    Circ_center = (420, 120)
    R = 60
    Xc, Yc = Circ_center
    obstacles = [
        (x >= 150 and x <= 175 and y <= 200 and y >= 100), 
        (x >= 250 and x <= 275 and y <= 100 and y >= 0),
        (((x - Xc)**2 + (y - Yc)**2) <= R**2),        
    ]
    return any(obstacles)

# Defining the clearance function
def clearance(x, y, clearance):
    clearance = clearance + robo_radius
    Circ_center = (420, 120)
    R = 60 + clearance
    Xc, Yc = Circ_center
    clearance_zones = [
        (x >= 150 - clearance and x <= 175 + clearance and y <= 200 + clearance  and y >= 100 - clearance),
        (x >= 250 - clearance and x <= 275 + clearance and y <= 100 + clearance and y >= 0 - clearance),
        (((x - Xc)**2 + (y - Yc)**2) <= R**2),
        (x <= clearance or x >= canvas_width - clearance or y <= clearance or y >= canvas_height - clearance),
    ]
    return any(clearance_zones)

# Function to check if the node is free
def is_free(x, y):
    return not (obstacles((x, y)) or clearance(x, y, clearance_distance))

# Drawing the obstacles on the canvas
for x in range(canvas_width):
    for y in range(canvas_height):
        if is_free(x, y):
            nodes.append((x, y))
        else:
            canvas[y, x] = obstacle_color

# Function for calculating the distance between two points
def distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

# Function for getting the nearest nodes
def nearest_nodes(tree, point, radius=20):
    return [node for node in tree if distance(node, point) < radius]

# Function for calculating the cost
def cost(tree, node):
    if node not in tree:
        return float('inf')
    total_cost = 0
    step = node
    while tree[step] is not None:
        total_cost += distance(step, tree[step])
        step = tree[step]
    return total_cost

# Function for extending the tree
def extend(tree, nearest, new_point, step_size=15):
    direction = np.array(new_point) - np.array(nearest)
    length = np.linalg.norm(direction)
    if length == 0:
        return None
    direction = direction / length
    current_length = min(step_size, length)
    new_node = tuple(np.array(nearest) + direction * current_length)
    new_node = tuple(map(int, new_node))
    return new_node

# Function for choosing the parent node
def choose_parent(tree, new_node, near_nodes):
    best_parent = None
    best_cost = float('inf')
    for node in near_nodes:
        if is_free(*new_node) and is_free(*node) and is_free_path(node, new_node) and cost(tree, node) + distance(node, new_node) < best_cost:
            best_parent = node
            best_cost = cost(tree, node) + distance(node, new_node)
    if best_parent:
        tree[new_node] = best_parent
        cv2.line(canvas, best_parent, new_node, path_color, 1)
    return tree

# Function for rewiring the tree
def rewire(tree, new_node, near_nodes):
    for node in near_nodes:
        if is_free(*new_node) and is_free(*node) and is_free_path(node, new_node) and cost(tree, new_node) + distance(new_node, node) < cost(tree, node):
            tree[node] = new_node
            cv2.line(canvas, new_node, node, path_color, 1)
    return tree

# Function for rewiring the goal node
def rewire_goal(tree, goal_node, near_nodes):
    for node in near_nodes:
        if is_free(*goal_node) and is_free(*node) and is_free_path(node, goal_node) and cost(tree, goal_node) + distance(goal_node, node) < cost(tree, node):
            tree[node] = goal_node
            cv2.line(canvas, goal_node, node, path_color, 1)
    return tree

# Function for checking if the path is free
def is_free_path(fr, to):
    x1, y1 = fr
    x2, y2 = to
    dx, dy = abs(x2 - x1), abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        if not is_free(x1, y1):
            return False
        if x1 == x2 and y1 == y2:
            break
        
        e2 =  2*err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx: 
            err += dx
            y1 += sy

    return True

# Function for RRT* algorithm
def RRT_star(start, goal, iterations=3000, search_radius=20):
    tree = {start: None}
    goal_node = None
    available_nodes = nodes.copy()
    time_costs =[]
    count = 0
    
    # Iterating for a number of iterations for RRT* 
    for _ in range(iterations):
        # Randomly choosing a point by checking is the random point is greater than 5
        if random.randint(0, 100) > 5:
                rand_point = (random.randint(0, canvas_width), random.randint(0, canvas_height))
        else:
            rand_point = goal
        
        # Getting the nearest node
        nearest = min(tree, key=lambda x: distance(x, rand_point))

        # Extending the tree
        new_node = extend(tree, nearest, rand_point)
        # Checking if the new node is not None
        if new_node:
            near_nodes = nearest_nodes(tree, new_node, search_radius)
            # Choosing the parent node
            tree = choose_parent(tree, new_node, near_nodes)
            # Rewiring the tree
            tree = rewire(tree, new_node, near_nodes)
            # Appending the time costs
            if distance(new_node, goal) < 10:
                if goal_node is None or cost(tree, new_node) < cost(tree, goal_node):
                    goal_node = new_node
                tree = rewire_goal(tree, goal_node, near_nodes)
        
    return tree, goal_node, time_costs

# Function for reconstructing the path
def reconstruct_path(tree, start, goal_node):
    path = []
    step = goal_node
    while step != start:
        path.append(step)
        step = tree[step]
    path.append(start)
    path.reverse()
    return path

# Class for PID Controller 
class PID_Control:
    # Initializing the PID Controller elements
    def __init__(self, Kp, Ki, Kd):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.last_error = 0
        self.integral = 0
    
    # Function for updating the error
    def update_error(self, error, dt):
        derivative = (error - self.last_error)/dt
        self.integral += error * dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output

# Class for the closed loop controller node
class CloseLoopControllerNode(Node):
    # Initializing the closed loop controller node
    def __init__(self, path):
        # Initializing the node
        super().__init__("Closed_loop_RRT_star")
        # Creating a publisher for the cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(
            Twist, "/cmd_vel",10)
        # Creating a subscriber for the pose
        self.pose_subscriber_ = self.create_subscription(
            Odometry, "/odom", self.pose_callback, 10)
        self.current_path = path
        self.current_pose = None
        self.path_i = 0
        self.linear = PID_Control(Kp = 0.5, Ki = 0, Kd = 0.0)
        self.angular = PID_Control(Kp = 0.7, Ki = 0, Kd = 0.0)

        self.get_logger().info("Controller has been initialized.")
    
    # Callback function for the pose
    def pose_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        quaternion = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        self.movement_update(yaw)
    
    # Function for updating the movement
    def movement_update(self, yaw):
        # Checking if the current pose and path are not None and Publishing the velocity
        if self.current_pose is None or self.current_path is None or self.path_i >= len(self.current_path):
            self.cmd_vel_pub.publish(Twist())
            return
        # Getting the current node and the next node
        current_node = (self.current_pose.position.x, self.current_pose.position.y)
        next_node = self.current_path[self.path_i]
        
        # Calculating the distance and angle to the next node
        dx = next_node[0] - current_node[0]
        dy = next_node[1] - current_node[1]
        distance_to_next_node = np.sqrt(dx**2 + dy**2)
        angle_to_next_node = np.arctan2(dy, dx)
        error_angle = angle_to_next_node - yaw
        dt = 0.1
        
        # Checking if the distance to the next node is less than 0.1 and updating the path index
        if distance_to_next_node < 0.1:
            self.path_i += 1
            self.get_logger().info(f"Moving to next waypoint: {next_node}")
            if self.path_i >= len(self.current_path):
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info("Goal Reached")
                return
        
        # Updating the error and publishing the velocities
        linear_velocity = self.linear.update_error(distance_to_next_node, dt) if distance_to_next_node > 0.1 else 0.0
        angular_velocity = self.angular.update_error(error_angle, dt) if distance_to_next_node > 0.1 else 0.0
        velocity = Twist()
        velocity.linear.x = linear_velocity
        velocity.angular.z = angular_velocity
        self.cmd_vel_pub.publish(velocity)

# Function for drawing the path
def convert_to_meters(path):
    return [((x -50)/ 100, (y -100) / 100) for x, y in path]

# Main function
def main(args=None):
    rclpy.init(args=args)
    start = (50, 100)
    goal = (550, 100)
    start_time = time.time()
    tree, last_node, _ = RRT_star(start, goal)
    if last_node:
        path = reconstruct_path(tree, start, last_node)
        print(path)
        print("Simplified Path: ")
        print(path)

        path_in_meters = convert_to_meters(path)
        print("Path in meters: ")
        print(path_in_meters)
    
    end_time = time.time()
    print("Time taken: ", end_time - start_time)
    
    node = CloseLoopControllerNode(path_in_meters)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
