import simpleguitk as simplegui
import random
import heapq
from threading import Timer

import pickle

# Constants
GRID_SIZE = 20
WIDTH = 600
HEIGHT = 600
ROWS = WIDTH // GRID_SIZE
COLS = HEIGHT // GRID_SIZE
ULTRASONIC_RANGE = 3  # Range of the ultrasonic sensor in grid units

# Colors
WHITE = 'White'
BLACK = 'Black'
RED = 'Red'
GREEN = 'Green'
BLUE = 'Blue'
YELLOW = 'Yellow'
LIGHT_BLUE = 'LightBlue'

# Initialize variables
start = None
goal = None
obstacles = set()
discovered_map = set()
current_path = []
q_table_initialized = False
training_done = False
current_index = 0
robot_position = None
sensor_readings = set()
episodes = 1000
epsilon_decay = 0.995
min_epsilon = 0.01
current_epsilon = 0.1

# Q-learning parameters
alpha = 0.1   # Learning rate
gamma = 0.9   # Discount factor

# Initialize Q-table
Q = {}

def initialize_q_table():
    global Q, q_table_initialized
    Q = {}
    for row in range(ROWS):
        for col in range(COLS):
            if (row, col) not in obstacles:
                Q[(row, col)] = {action: 0 for action in ['up', 'down', 'left', 'right']}
    q_table_initialized = True

def load_q_table():
    global Q
    try:
        with open('q_table.pkl', 'rb') as f:
            Q = pickle.load(f)
    except FileNotFoundError:
        initialize_q_table()

def save_q_table():
    global Q
    with open('q_table.pkl', 'wb') as f:
        pickle.dump(Q, f)

# Dijkstra's Algorithm for initial path planning
def dijkstra(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        for neighbor in get_neighbors(current):
            tentative_g_score = g_score[current] + 1
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                heapq.heappush(open_set, (tentative_g_score, neighbor))
    
    return []

def get_neighbors(node):
    neighbors = [
        (node[0] + 1, node[1]), (node[0] - 1, node[1]),
        (node[0], node[1] + 1), (node[0], node[1] - 1)
    ]
    return [n for n in neighbors if 0 <= n[0] < ROWS and 0 <= n[1] < COLS and n not in obstacles]

# Q-learning functions
def get_action(state):
    global current_epsilon
    if random.uniform(0, 1) < current_epsilon:
        return random.choice(['up', 'down', 'left', 'right'])
    else:
        return max(Q[state], key=Q[state].get)

def get_next_state(state, action):
    if action == 'up':
        return (state[0] - 1, state[1])
    elif action == 'down':
        return (state[0] + 1, state[1])
    elif action == 'left':
        return (state[0], state[1] - 1)
    elif action == 'right':
        return (state[0], state[1] + 1)

def get_reward(state):
    if state == goal:
        return 100
    else:
        return -1

def q_learning_step():
    state = start
    while state != goal:
        action = get_action(state)
        next_state = get_next_state(state, action)
        
        if next_state in obstacles or not (0 <= next_state[0] < ROWS) or not (0 <= next_state[1] < COLS):
            next_state = state
        
        if next_state not in Q:
            Q[next_state] = {action: 0 for action in ['up', 'down', 'left', 'right']}
        
        reward = get_reward(next_state)
        old_value = Q[state][action]
        next_max = max(Q[next_state].values())
        
        new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)
        Q[state][action] = new_value
        
        state = next_state

def run_q_learning():
    global training_done, current_epsilon
    if start is None or goal is None:
        print("Start or goal not set!")
        return
    load_q_table()
    for episode in range(episodes):
        q_learning_step()
        current_epsilon = max(min_epsilon, current_epsilon * epsilon_decay)
    save_q_table()
    training_done = True

def get_learned_path():
    if not training_done:
        return []
    state = start
    path = [state]
    while state != goal:
        action = get_action(state)
        state = get_next_state(state, action)
        path.append(state)
    return path

def mouse_click(pos):
    global start, goal, current_path, robot_position
    row, col = pos[1] // GRID_SIZE, pos[0] // GRID_SIZE
    if pos[0] >= WIDTH or pos[1] >= HEIGHT:
        return
    if start is None:
        start = (row, col)
        robot_position = start
    elif goal is None:
        goal = (row, col)
    elif (row, col) not in obstacles:
        obstacles.add((row, col))
    else:
        obstacles.remove((row, col))
    current_path = []

def draw(canvas):
    global current_index, current_path, robot_position, sensor_readings
    for row in range(ROWS):
        for col in range(COLS):
            color = WHITE
            if (row, col) in obstacles:
                if (row, col) in sensor_readings:
                    color = YELLOW  # Highlight detected obstacles in yellow
                else:
                    color = BLACK
            elif (row, col) == start:
                color = GREEN
            elif (row, col) == goal:
                color = RED
            elif (row, col) == robot_position:
                color = BLUE  # Use blue color for the robot position
            elif (row, col) in current_path:
                color = LIGHT_BLUE
            elif (row, col) in discovered_map:
                color = LIGHT_BLUE
            canvas.draw_polygon([(col * GRID_SIZE, row * GRID_SIZE), 
                                 ((col + 1) * GRID_SIZE, row * GRID_SIZE),
                                 ((col + 1) * GRID_SIZE, (row + 1) * GRID_SIZE),
                                 (col * GRID_SIZE, (row + 1) * GRID_SIZE)], 
                                1, 'Gray', color)
        
    
   
def reset():
    global start, goal, obstacles, current_path, q_table_initialized, training_done, current_index, robot_position, discovered_map, sensor_readings
    start = None
    goal = None
    obstacles = set()
    current_path = []
    q_table_initialized = False
    training_done = False
    current_index = 0
    robot_position = None
    discovered_map = set()
    sensor_readings = set()

def simulate():
    global current_index, robot_position, discovered_map, sensor_readings
    if current_path and current_index < len(current_path) - 1:
        current_index += 1
        robot_position = current_path[current_index]
        discovered_map.add(robot_position)
        sensor_readings = detect_obstacles(robot_position)
        print(f"Sensor readings at position {robot_position}: {sensor_readings}")
        # Schedule the next step after a delay (adjust delay as needed)
        timer = Timer(0.5, simulate)
        timer.start()
    else:
        print("Reached the goal!")
        sensor_readings = set()  # Clear sensor readings after simulation

def start_simulation():
    global current_index, current_path, robot_position
    if start is None or goal is None:
        print("Start or goal not set!")
        return
    if not training_done:
        run_q_learning()
    current_path = get_learned_path()
    current_index = 0
    robot_position = start
    # Call simulate to start the automatic execution
    simulate()


import time

def step():
    global current_index, robot_position, discovered_map, sensor_readings
    if current_path and current_index < len(current_path) - 1:
        current_index += 1
        robot_position = current_path[current_index]
        discovered_map.add(robot_position)
        sensor_readings = detect_obstacles(robot_position)
        print(f"Sensor readings at position {robot_position}: {sensor_readings}")
        time.sleep(0.5)  # Delay for 0.5 seconds (adjust as needed)
    else:
        print("Reached the goal!")
    sensor_readings = set()  # Clear sensor readings after each step

def detect_obstacles(position):
    global discovered_map
    readings = set()
    for row_offset in range(-ULTRASONIC_RANGE, ULTRASONIC_RANGE + 1):
        for col_offset in range(-ULTRASONIC_RANGE, ULTRASONIC_RANGE + 1):
            neighbor = (position[0] + row_offset, position[1] + col_offset)
            if 0 <= neighbor[0] < ROWS and 0 <= neighbor[1] < COLS and neighbor not in discovered_map:
                discovered_map.add(neighbor)
                if neighbor in obstacles:
                    readings.add(neighbor)
    return readings

# Create frame
frame = simplegui.create_frame("Robot Path Planning with Q-learning and SLAM", WIDTH, HEIGHT)
frame.set_draw_handler(draw)
frame.set_mouseclick_handler(mouse_click)

# Add buttons
frame.add_button("Reset", reset, 100)
frame.add_button("Start Simulation", start_simulation, 100)


# Start frame
frame.start()

