# Importing necessary libraries
import cv2
import numpy as np
from queue import PriorityQueue
import time
import random

# Canvas dimensions
canvas_height = 500
canvas_width = 500

# Defining the colors
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
    y_transform = abs(y - canvas_height)
    obstacles = [
        (x >= 115 and x <= 135  and y_transform >= 125 and y_transform <= 375), 
        (x >= 135 and x <= 260 and y_transform >= 240 and y_transform <= 260 ),
        (x >= 240 and x <= 260 and y_transform >= 0 and y_transform <= 240),
        (x >= 240 and x <= 365  and y_transform >= 355 and y_transform <= 375),
        (x >= 365 and x <= 385 and y_transform >= 125 and y_transform <= 500 ),

    ]
    return any(obstacles)

# Function for checking if the node is free
def is_free(x, y):
    return not obstacles((x, y)) 
 
# Drawing the obstacles on the canvas
for i in range(canvas_width):
    for j in range(canvas_height):
        if is_free(i, j):
            nodes.append((i, j))
        else:
            canvas[j, i] = obstacle_color
            obs.add((i, j))

# Function for calculating the distance between two points
def distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

# Function for getting the nearest node
def nearest_node(tree, point):
    return min(tree, key=lambda x: distance(x, point))

# Function for extending the tree
def extend(tree, nearest, new_point, step_size=10):
    direction = np.array(new_point) - np.array(nearest)
    length = np.linalg.norm(direction)
    if length == 0:
        return None  
    direction = direction / length
    current_length = min(step_size, length)
    new_node = tuple(np.array(nearest) + direction * current_length)
    new_node = tuple(map(int, new_node))

    if is_free(*new_node):  
        tree[new_node] = nearest
        cv2.line(canvas, tuple(map(int, nearest)), new_node, path_color, 1)  
        return new_node
    return None
# Function for RRT algorithm
def RRT(start, goal, iterations=5000):
    # Initializing the tree
    tree = {start: None}
    # Initializing the available nodes
    available_nodes = nodes.copy()
    # Iterating for a number of iterations for RRT
    for _ in range(iterations):
        out.write(canvas)
        # Randomly choosing a point by checking is the random point is greater than 5
        if random.randint(0, 100) > 5:
            if available_nodes:
                rand_point = random.randint(0, len(available_nodes) - 1)
                rand_point = available_nodes.pop(rand_point)
            else:
                break
            
        else:
            rand_point = goal
        nearest = nearest_node(tree, rand_point)
        new_node = extend(tree, nearest, rand_point)
        if new_node and distance(new_node, goal) < 10: 
            return tree, new_node
    return tree, None

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

# Function for calculating the path cost
def calculate_path_cost(path):
    total_cost = 0
    for i in range(len(path) - 1):
        total_cost += distance(path[i], path[i + 1])
    return total_cost

# Function for drawing the path
def draw_path(path, color=(255, 0, 0)):
    for i in range(len(path) - 1):
        cv2.line(canvas, path[i], path[i + 1], color, 2)  
        out.write(canvas)

# Function for getting the parent nodes
def inputs():
    print("RRT")
    Xin = int(input("Enter the x-coordinate of the initial point: "))
    Yin = int(input("Enter the y-coordinate of the initial point: "))
    Xf = int(input("Enter the x-coordinate of the goal point: "))
    Yf = int(input("Enter the y-coordinate of the goal point: "))
    return Xin, Yin, Xf, Yf
# Getting the inputs
Xin, Yin, Xf, Yf = inputs()
# Checking if the inputs are valid
valid = False
# While not valid ask for new inputs
while not valid:
    if is_free(Xin, abs(Yin - canvas_height)) and is_free(Xf, abs(Yf - canvas_height)) and 0 <= Xin < canvas_width and 0 <= Xf < canvas_width and 0 <= Yin < canvas_height and 0 <= Yf < canvas_height:
        valid = True
    else:
        print("Invalid start or goal point. Please try again.")
        Xin, Yin, Xf, Yf = inputs()

# Initializing the start and goal points
start = (Xin, abs(Yin - canvas_height))
goal = (Xf, abs(Yf - canvas_height))

# Initializing the video writer
out = cv2.VideoWriter('RRT.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (canvas_width, canvas_height))

# Running the RRT algorithm
start_time = time.time()
# Drawing the start and goal points
cv2.circle(canvas, start, 5, (0, 0, 255), -1)
cv2.circle(canvas, goal, 5, (0, 255, 0), -1)

# Drawing the obstacles on the canvas
for x in range(canvas_width):
    for y in range(canvas_height):
        if is_free(x, y):
            nodes.append((x, y))
        else:
            canvas[y, x] = obstacle_color

# Running the RRT algorithm
tree, last_node = RRT(start, goal)
if last_node:
    path = reconstruct_path(tree, start, last_node)
    draw_path(path)
    for i in range(30):
        out.write(canvas)
    path_cost = calculate_path_cost(path)
    print("Path cost: ", path_cost)
   
end_time = time.time()
print("Time taken: ", end_time - start_time)

# Displaying the final path
cv2.imshow("Path Planning with RRT", canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()
