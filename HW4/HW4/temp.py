import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# Parameters
num_robots = 5
goal = np.array([10, 10])  # Goal position
obstacles = np.array([[5, 5], [8, 12], [12, 7]])  # Obstacle positions
k_goal = 1.0
k_obst = 1.0
k_form = 1.0

# Define the function to compute the derivative of positions
def compute_derivative(robot_positions, t):
    # Ensure robot_positions has shape (num_robots, 2) and goal has shape (1, 2)
    robot_positions = robot_positions.reshape(num_robots, 2)
    goal_position = np.array([goal])  # Reshape goal to have shape (1, 2)
    
    # Compute forces
    F_goal = -k_goal * (robot_positions - goal_position)
    F_obst = np.zeros_like(robot_positions)
    for obstacle in obstacles:
        distance = robot_positions - obstacle
        distance_norm = np.linalg.norm(distance, axis=1)
        F_obst += k_obst * (distance / distance_norm[:, None]**3)
    
    # Formation maintenance forces
    F_form = -k_form * (robot_positions - desired_formation_positions)
    
    # Compute derivatives
    derivatives =F_form
    return derivatives.flatten()

# Initialize robot positions randomly
robot_positions = np.random.rand(num_robots, 2) * 15

# Define desired formation positions (e.g., linear formation)
desired_formation_positions = np.array([[i, 0] for i in range(num_robots)])

# Time array
t = np.linspace(0, 10, 10)  # From 0 to 10 seconds with 100 time points

# Integrate ODEs
solution = odeint(compute_derivative, robot_positions.flatten(), t)
solution = solution.reshape(-1, num_robots, 2)

# Plotting
for i in range(len(t)):
    plt.figure()
    plt.scatter(goal[0], goal[1], color='g', marker='x', label='Goal')
    plt.scatter(obstacles[:, 0], obstacles[:, 1], color='r', marker='o', label='Obstacles')
    plt.scatter(desired_formation_positions[:, 0], desired_formation_positions[:, 1], color='orange', marker='s', label='Desired Formation')
    plt.scatter(solution[i, :, 0], solution[i, :, 1], color='b', marker='o', label='Robots')
    plt.title(f'Time: {t[i]:.2f}')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
plt.show()
