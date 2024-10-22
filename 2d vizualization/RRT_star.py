# Import the necessary libraries
import cv2
import numpy as np
from queue import PriorityQueue
import time
import random
import matplotlib.pyplot as plt

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

# Initializing a white canvas
canvas = np.ones((canvas_height, canvas_width, 3), dtype="uint8") * 255

# Defining the obstacles 
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

# Function for calculating the cost to reach a node
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

# Function for Runnning the RRT* algorithm
def RRT_star(start, goal, iterations=3000, search_radius=20, step_size=10):
    # Initializing the tree
    tree = {start: None}
    # Initializing the goal node
    goal_node = None
    # Initializing the available nodes
    available_nodes = nodes.copy()
    # Initializing the time costs
    time_costs =[]
    # Initializing the count
    count = 0
    
    # Iterating over the number of iterations for the RRT* algorithm
    for u in range(iterations):
        if iterations % 10 == 0:
            out.write(canvas)
        # Generating a random point by checking if the random point is greater than 5
        if random.randint(0, 100) > 5:
                rand_point = (random.randint(0, canvas_width), random.randint(0, canvas_height))
        else:
            rand_point = goal
        # Getting the nearest node    
        nearest = min(tree, key=lambda x: distance(x, rand_point))
        # Extending the tree
        new_node = extend(tree, nearest, rand_point, step_size)
        # Checking if the new node is not None
        if new_node:
            near_nodes = nearest_nodes(tree, new_node, search_radius)
            tree = choose_parent(tree, new_node, near_nodes)
            tree = rewire(tree, new_node, near_nodes)
            # Checking if the distance between the new node and the goal is less than 10
            if distance(new_node, goal) < 10:
                if count == 0:
                    tg = time.time()
                    time_to_goal = tg - start_time
                    current_cost = cost(tree, new_node)
                    print(f"Time to goal: {time_to_goal}, Cost to goal: {current_cost}")

                    count += 1
                if goal_node is None or cost(tree, new_node) < cost(tree, goal_node):
                    goal_node = new_node
                tree = rewire_goal(tree, goal_node, near_nodes)
        current_time = time.time() - start_time
        current_cost = cost(tree, goal_node) if goal_node else float('inf')
        time_costs.append((current_time, current_cost))
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

# Function for drawing the path
def draw_path(path):
    for i in range(len(path) - 1):
        cv2.line(canvas, path[i], path[i + 1], (255, 0, 0), 2)  
        out.write(canvas)

# Function for getting the inputs
def inputs():
    print("RRT* Path Planning")
    Xi = input("Enter the x-coordinate of the start point: ")
    Yi = input("Enter the y-coordinate of the start point: ")
    Xf = input("Enter the x-coordinate of the goal point: ")
    Yf = input("Enter the y-coordinate of the goal point: ")
    IT = input("Enter the number of iterations: ")
    ST = input("Enter the step size: ")
    SR = input("Enter the search radius: ")
    return int(Xi), int(Yi), int(Xf), int(Yf), int(IT), int(ST), int(SR)

# Getting the inputs
Xi, Yi, Xf, Yf, IT, ST, SR = inputs()

# Checking if the start and goal points are valid
valid = False

# while not valid ask for new inputs
while not valid:
    if not is_free(Xi, abs(Yi - canvas_height)) or not is_free(Xf, abs(Yf - canvas_height)) or not (0 <= Xi < canvas_width) or not (0 <= Xf < canvas_width) or not (0 <= Yi < canvas_height) or not (0 <= Yf < canvas_height):
        print("Invalid start or goal point. Try again.")
        Xi, Yi, Xf, Yf, IT, ST, SR = inputs()
    else:
        valid = True

# Initializing the start and goal points
start = (Xi, abs(Yi - canvas_height))  
goal = (Xf, abs(Yf - canvas_height))  

# Initializing the start time
out = cv2.VideoWriter(f'RRT_star.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (canvas_width, canvas_height))

# Drawing the start and goal points
cv2.circle(canvas, start, 5, (0, 0, 255), -1)
cv2.circle(canvas, goal, 5, (0, 255, 0), -1)

# Running the RRT* algorithm
start_time = time.time()
tree, last_node, time_costs = RRT_star(start, goal, IT, SR, ST)
end_time = time.time()
if last_node:
    path = reconstruct_path(tree, start, last_node)
    path_cost = cost(tree, last_node)
    print("Path cost: ", path_cost)
    draw_path(path)
    for i in range(30):
        out.write(canvas)
print("Time taken: ", end_time - start_time)

# Displaying the final output
times, costs = zip(*time_costs)
plt.figure(figsize=(10, 5))
plt.plot(times, costs, marker='o', markersize=1, label="RRT*")
plt.xlabel('Time (s)')
plt.ylabel('Cost to Goal')
plt.title('Cost to Goal over Time')
plt.grid(True)
plt.show()

# Showing the path planning
cv2.imshow("Path Planning with RRT*", canvas)
cv2.waitKey(0)
out.release()
cv2.destroyAllWindows()
