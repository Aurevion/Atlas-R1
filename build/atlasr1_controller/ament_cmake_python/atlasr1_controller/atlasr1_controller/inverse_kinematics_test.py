import numpy as np

def calculate_omni_jacobian(robot_orientation_theta, wheel_angles, wheel_radius, robot_radius):
    """
    Calculates the 3x3 inverse kinematics Jacobian matrix for a 3-wheeled 
    holonomic robot based on its current orientation (theta) in the world frame.

    Args:
        robot_orientation_theta (float): The current yaw angle (orientation) of the 
                                         robot in radians (e.g., from an IMU or odometry).
        wheel_angles (list): A list of the fixed physical angles of each wheel 
                             relative to the robot's chassis center (in radians).
        wheel_radius (float): The radius of the omni-wheel.
        robot_radius (float): The distance from the center of the robot to each wheel.

    Returns:
        np.array: The 3x3 Jacobian (inverse kinematics) matrix.
    """
    
    J_inv = []
    
    # Iterate through each wheel's configuration
    for alpha_i in wheel_angles:
        # The direction the wheel drives (drive_angle) is typically alpha_i + pi/2 
        # relative to the robot's local frame. 
        # The Jacobian formulation typically uses the angle of the wheel's axis. 
        # For a standard symmetric 3-wheel configuration, the inverse kinematic 
        # matrix elements follow a specific pattern.
        
        # J_inv maps [Vx, Vy, Omega] -> [WheelSpeed_i]
        # Formula for row i: 
        # [ (-sin(theta + alpha_i)) / r , (cos(theta + alpha_i)) / r , robot_radius / r ]

        row = [
            (-np.sin(robot_orientation_theta + alpha_i)) / wheel_radius,
            (np.cos(robot_orientation_theta + alpha_i)) / wheel_radius,
            robot_radius / wheel_radius
        ]
        J_inv.append(row)
        
    return np.array(J_inv)

# --- Example Usage ---

# Assuming a standard symmetric configuration: 0, 120, and 240 degrees (or 0, 2pi/3, 4pi/3 radians)
# Alpha angles define the direction of the motor axis relative to the robot X-axis.
WHEEL_ANGLES = [0.0, 2.0 * np.pi / 3.0, 4.0 * np.pi / 3.0] # Radians
WHEEL_RADIUS = 0.0019 # meters (5 cm)
ROBOT_RADIUS = 0.774  # meters (20 cm, distance from center to wheel)

# Example: Robot is oriented at 45 degrees (pi/4 radians) in the world frame
current_robot_yaw = np.pi/2

# Generate the Jacobian matrix
jacobian_matrix = calculate_omni_jacobian(
    current_robot_yaw, 
    WHEEL_ANGLES, 
    WHEEL_RADIUS, 
    ROBOT_RADIUS
)

print("Generated 3x3 Jacobian (Inverse Kinematics) Matrix:")
print(jacobian_matrix)

# Example of how to use it:
# If you want the robot to move with a linear X speed of 0.1 m/s and rotate at 0.5 rad/s:
desired_robot_velocities = np.array([1.0, 0, 0.5]) # [Vx, Vy, Omega]

# Calculate required wheel speeds using matrix multiplication
wheel_speeds = np.matmul(jacobian_matrix, desired_robot_velocities)

print("\nDesired Robot Velocities (Vx, Vy, Omega):", desired_robot_velocities)
print("Required Wheel Speeds (rad/s):")
print(wheel_speeds)
