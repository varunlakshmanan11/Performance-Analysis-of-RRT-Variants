# importing the required libraries
import cv2
import numpy as np
import time
import random
import math
import matplotlib.pyplot as plt

# Canvas dimensions
canvas_height = 500
canvas_width = 500

# Define the colors
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

# Drawing the obstacles on the canvas
def obstacles(node):
    x, y = node
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

# loop to check if the node is free and append it to the nodes list
for x in range(canvas_width):
    for y in range(canvas_height):
        if is_free(x, y):
            nodes.append((x, y))
        else:
            canvas[y, x] = obstacle_color

# Function for calculating the distance between two points
def distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

# Function for calculating if the node is within the canvas
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
    if length < 1:
        return None
    direction = direction / length
    current_length = min(step_size, length)
    new_node = tuple(np.array(nearest) + direction * current_length)
    new_node = tuple(map(int, new_node))
    if is_free(*new_node) and is_free_path(nearest, new_node):
        return new_node
    return None

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

# Function for choosing the parent node
def choose_parent(tree, new_node, near_nodes):
    best_parent = None
    best_cost = float('inf')
    for node in near_nodes:
        if is_free_path(node, new_node) and is_free(*node) and cost(tree, node) + distance(node, new_node) < best_cost:
            best_parent = node
            best_cost = cost(tree, node) + distance(node, new_node)
    if best_parent:
        tree[new_node] = best_parent
        cv2.line(canvas, best_parent, new_node, path_color, 1)
    return tree

# Function for getting the parent nodes
def get_parent_nodes(tree, node, depth):
    parents = []
    current = node
    while depth > 0 and tree.get(current) is not None:
        parent = tree[current]
        parents.append(parent)
        current = parent
        depth -= 1
    return parents[::-1] 

# Function for rewiring the tree
def q_rewire(tree, new_node, near_nodes_with_ancestry, ad):
    for node in near_nodes_with_ancestry:
        for  x_from in [new_node] + get_parent_nodes(tree, new_node, ad):
            sigma = extend(tree, x_from, node)
            if sigma and is_free(*sigma) and is_free_path(x_from, node) and cost(tree, x_from) + distance(x_from, sigma) < cost(tree, node) :
                tree[node] = x_from
                cv2.line(canvas, x_from, node, path_color, 1)
    return tree

# Function for sampling points in an ellipse
def sample_points_in_ellipse(start, goal, num_points, C):
    a = distance(start, goal) / 2
    b = math.sqrt(C**2 - distance(start, goal)**2)/2   # Minor axis half-length
    center = ((start[0] + goal[0]) / 2, (start[1] + goal[1]) / 2)
    theta = math.atan2(goal[1] - start[1], goal[0] - start[0])
    # Generating random angles
    angles = np.random.uniform(0, 2 * np.pi, num_points)
    # Generating random radii
    radii = np.sqrt(np.random.uniform(0, 1, num_points))
    # Calculating x, y coordinates in the unit circle
    x = radii * np.cos(angles)
    y = radii * np.sin(angles)
    # Scaling by ellipse axes
    x *= a
    y *= b
    # Rotating points by theta
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    x_rot = cos_theta * x - sin_theta * y
    y_rot = sin_theta * x + cos_theta * y
    # Translating to center
    x_final = x_rot + center[0]
    y_final = y_rot + center[1]
    return x_final, y_final

# Function for the Informed Quick RRT* algorithm
def I_Q_RRT_star(start, goal, iterations=2000, search_radius=20, ad=1, step_size=10):
    # Checking if the goal is reached
    GOAL_REACHED = False
    # Initializing the tree
    tree = {start: None}
    # Initializing the goal node
    goal_node = None
    # Initializing the available nodes
    available_nodes = nodes.copy()
    # Initializing the cost to the goal
    time_costs =[]
    # Initializing the count
    count = 0
    
    # Looping through the iterations for the Informed Quick RRT* algorithm
    for u in range(iterations):
        if iterations % 10 == 0:
            out.write(canvas)
        # Checking if the goal is reached and sampling points in the ellipse to get the random point.
        if GOAL_REACHED:
            x_final, y_final = sample_points_in_ellipse(start, goal_node, num_points=1000, C= cost_n)
            valid_points = [(x, y) for x, y in zip(x_final, y_final) if is_free(x, y)]
            if valid_points:
                rand_point = random.choice(valid_points)
            else:
                continue
        # checking if the random point is greater than 5
        if random.randint(0, 100) > 5:
            if available_nodes:
                rand_point = random.randint(0, len(available_nodes) - 1)
                rand_point = available_nodes.pop(rand_point)
            else:
                break
        else:
            rand_point = goal
        # Getting the nearest node
        nearest = min(tree, key=lambda x: distance(x, rand_point))
        # Extending the tree
        new_node = extend(tree, nearest, rand_point, step_size)
        # Checking if the new node is not None
        if new_node:
            near_nodes = nearest_nodes(tree, new_node, search_radius)
            for n in near_nodes:
                ancestor_nodes = get_parent_nodes(tree, n, ad)
                near_nodes_with_ancestry = near_nodes + ancestor_nodes
            tree = choose_parent(tree, new_node, near_nodes_with_ancestry)
            tree = q_rewire(tree, new_node, near_nodes_with_ancestry, ad)
            # Checking if the distance between the new node and the goal is less than 10
            if distance(new_node, goal) < 10:
                GOAL_REACHED = True
                if count == 0:
                    tg = time.time()
                    time_to_goal = tg - start_time
                    current_cost = cost(tree, new_node)
                    print(f"Time to goal: {time_to_goal}, Cost to goal: {current_cost}")
                    count += 1
                if goal_node is None or cost(tree, new_node) < cost(tree, goal_node):
                    goal_node = new_node
                tree = q_rewire(tree, goal_node, near_nodes_with_ancestry, ad)
                cost_n = cost(tree, new_node)
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

# Intializing the time, cost and depth a empty lists
times = []
costs = []
depths = []

# Function for getting the inputs
def inputs():
    print(("Informed + Quick RRT*"))
    Xin = int(input("Enter the x-coordinate of the initial point: "))
    Yin = int(input("Enter the y-coordinate of the initial point: "))
    Xf = int(input("Enter the x-coordinate of the goal point: "))
    Yf = int(input("Enter the y-coordinate of the goal point: "))
    AD = int(input("Enter the depth: "))
    IT = int(input("Enter the number of iterations: "))
    ST = int(input("Enter the step size: "))
    SR = int(input("Enter the search radius: "))
    return Xin, Yin, Xf, Yf, AD, IT, ST, SR

# Getting the inputs
Xin, Yin, Xf, Yf, AD, IT, ST, SR = inputs()
# Checking if the inputs are valid
valid = False
# Looping through the inputs
while not valid:
    if is_free(Xin, abs(Yin - canvas_height)) and is_free(Xf, abs(Yf - canvas_height)) and 0 <= Xin < canvas_width and 0 <= Xf < canvas_width and 0 <= Yin < canvas_height and 0 <= Yf < canvas_height:
        valid = True
    else:
        print("Invalid start or goal point. Please try again.")
        Xin, Yin, Xf, Yf, AD, IT, ST, SR = inputs()
# Initializing the canvas
canvas = np.ones((canvas_height, canvas_width, 3), dtype="uint8") * 255
nodes = []
for x in range(canvas_width):
    for y in range(canvas_height):
        if is_free(x, y):
            nodes.append((x, y))
        else:
            canvas[y, x] = obstacle_color 

# Initializing the video writer
out = cv2.VideoWriter('I_Q_RRT_star.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (canvas_width, canvas_height))

# Initializing the start and goal points
start = (Xin, abs(Yin - canvas_height))
goal = (Xf, abs(Yf - canvas_height))

# Drawing the start and goal points
cv2.circle(canvas, start, 5, (0, 0, 255), -1)
cv2.circle(canvas, goal, 5, (0, 255, 0), -1)
out.write(canvas)

# Initializing the path cost
path_cost = 0

# Running the Informed Quick RRT* algorithm
start_time = time.time()
tree, last_node, time_costs = I_Q_RRT_star(start, goal, IT, SR, AD, ST)
end_time = time.time()
if last_node:
    path = reconstruct_path(tree, start, last_node)
    draw_path(path)
    path_cost = cost(tree, last_node)
else:
    path_cost = float('inf')


time_taken = end_time - start_time

# Printing the results
print(f"{IT} {AD} Path cost: {path_cost}")
print(f"{IT} {AD} Time taken: {end_time - start_time}")

# Plotting the cost to goal over time
times, costs = zip(*time_costs)
plt.figure(figsize=(10, 5))
plt.plot(times, costs, marker='o', markersize=1, label=f"Depth: {AD}")
plt.xlabel('Time (s)')
plt.ylabel('Cost to Goal')
plt.title('Cost to Goal over Time')
plt.grid(True)
plt.show()

# Showing the path planning
cv2.imshow("Path Planning with Informed Quick-RRT*", canvas)
cv2.waitKey(0)
out.release()

