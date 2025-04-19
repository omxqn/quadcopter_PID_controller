# Implement a simplified controller for UAV position stabilization
# AERO60492 - Autonomous Mobile Robots - Coursework 3
import numpy as np
from math import sin, cos

def rotation_matrix_z(yaw):
    """
    Create a rotation matrix for rotation around the z-axis (yaw)
    
    Args:
        yaw: Rotation angle in radians
        
    Returns:
        3x3 rotation matrix
    """
    return np.array([
        [cos(yaw), -sin(yaw), 0],
        [sin(yaw), cos(yaw), 0],
        [0, 0, 1]
    ])

def controller(state, target_pos, dt):
    """
    Simple PID controller for UAV position stabilization
    
    Args:
        state: [position_x (m), position_y (m), position_z (m), roll (radians), pitch (radians), yaw (radians)]
        target_pos: (x (m), y (m), z (m), yaw (radians))
        dt: time step (s)
    
    Returns:
        velocity command: (velocity_x_setpoint (m/s), velocity_y_setpoint (m/s), velocity_z_setpoint (m/s), yaw_rate_setpoint (radians/s))
    """
    # Extract current state
    current_pos = np.array(state[:3])  # x, y, z position
    current_yaw = state[5]  # yaw angle
    
    # Extract target state
    target_pos_xyz = np.array(target_pos[:3])  # x, y, z position
    target_yaw = target_pos[3]  # target yaw
    
    # Calculate position error in global frame
    pos_error_global = target_pos_xyz - current_pos
    
    # Initialize controller parameters (if they don't exist)
    if not hasattr(controller, 'initialized'):
        # Position control gains
        controller.Kp_xy = 0.8  # Proportional gain for x,y
        controller.Ki_xy = 0.05  # Integral gain for x,y
        controller.Kd_xy = 0.2  # Derivative gain for x,y
        
        controller.Kp_z = 1.0  # Proportional gain for z
        controller.Ki_z = 0.1  # Integral gain for z
        controller.Kd_z = 0.2  # Derivative gain for z
        
        # Yaw control gain (P control only for simplicity)
        controller.Kp_yaw = 1.0
        
        # Initialize integral and previous error terms
        controller.integral_error = np.zeros(3)
        controller.prev_error = np.zeros(3)
        
        controller.initialized = True
    
    # Update integral term with anti-windup
    controller.integral_error = controller.integral_error + pos_error_global * dt
    
    # Apply anti-windup - limit integral term
    max_integral = np.array([1.0, 1.0, 1.0])
    controller.integral_error = np.clip(controller.integral_error, -max_integral, max_integral)
    
    # Calculate derivative term
    derivative = (pos_error_global - controller.prev_error) / dt
    controller.prev_error = pos_error_global.copy()
    
    # Apply PID control for position
    # Different gains for xy and z
    p_term = np.zeros(3)
    i_term = np.zeros(3)
    d_term = np.zeros(3)
    
    # XY control
    p_term[0:2] = controller.Kp_xy * pos_error_global[0:2]
    i_term[0:2] = controller.Ki_xy * controller.integral_error[0:2]
    d_term[0:2] = controller.Kd_xy * derivative[0:2]
    
    # Z control
    p_term[2] = controller.Kp_z * pos_error_global[2]
    i_term[2] = controller.Ki_z * controller.integral_error[2]
    d_term[2] = controller.Kd_z * derivative[2]
    
    # Calculate velocity command in global frame
    vel_cmd_global = p_term + i_term + d_term
    
    # Transform velocity command from global to body frame
    # This is crucial - the drone expects velocity commands in its body frame
    R = rotation_matrix_z(-current_yaw)
    vel_cmd_body = R @ vel_cmd_global
    
    # Calculate yaw error and normalize to [-pi, pi]
    yaw_error = target_yaw - current_yaw
    while yaw_error > np.pi:
        yaw_error -= 2 * np.pi
    while yaw_error < -np.pi:
        yaw_error += 2 * np.pi
    
    # Simple P control for yaw
    yaw_rate_cmd = controller.Kp_yaw * yaw_error
    
    # Limit velocity commands to reasonable values
    vel_cmd_body = np.clip(vel_cmd_body, -1.0, 1.0)
    yaw_rate_cmd = np.clip(yaw_rate_cmd, -1.0, 1.0)
    
    # Return velocity command in the format expected by the simulator
    output = (float(vel_cmd_body[0]), float(vel_cmd_body[1]), float(vel_cmd_body[2]), float(yaw_rate_cmd))
    
    return output
