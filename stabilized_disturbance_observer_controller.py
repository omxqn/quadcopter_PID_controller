# Stabilized Disturbance Observer-based Controller for UAV position stabilization
# AERO60492 - Autonomous Mobile Robots - Coursework 3
import numpy as np
from math import sin, cos

class PIDController:
    """
    PID Controller implementation with anti-windup protection
    """
    def __init__(self, Kp, Ki, Kd, Ki_sat, output_limits=None):
        """
        Initialize PID controller with gain values and saturation limits
        
        Args:
            Kp: Proportional gain
            Ki: Integral gain
            Kd: Derivative gain
            Ki_sat: Integral term saturation limit
            output_limits: Optional tuple (min, max) for output limiting
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ki_sat = Ki_sat
        self.output_limits = output_limits
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_output = 0.0
    
    def reset(self):
        """Reset controller state (integral term and previous error)"""
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_output = 0.0
    
    def update(self, error, dt):
        """
        Update controller and calculate output based on current error
        
        Args:
            error: Current error value (setpoint - measurement)
            dt: Time step in seconds
            
        Returns:
            Control output value
        """
        # Update integral term with anti-windup protection
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.Ki_sat, self.Ki_sat)
        
        # Calculate derivative term (avoid division by zero)
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        self.previous_error = error
        
        # Calculate controller output
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # Apply output limits if specified
        if self.output_limits is not None:
            output = np.clip(output, self.output_limits[0], self.output_limits[1])
        
        self.last_output = output
        return output

class LowPassFilter:
    """
    First-order low-pass filter for smoothing signals
    """
    def __init__(self, cutoff_freq, initial_value=0.0):
        """
        Initialize low-pass filter
        
        Args:
            cutoff_freq: Cutoff frequency in Hz
            initial_value: Initial value of the filter
        """
        self.cutoff_freq = cutoff_freq
        self.prev_output = initial_value if isinstance(initial_value, np.ndarray) else np.array([initial_value])
        
    def update(self, input_value, dt):
        """
        Update filter with new input value
        
        Args:
            input_value: New input value
            dt: Time step in seconds
            
        Returns:
            Filtered output value
        """
        # Calculate filter coefficient
        alpha = dt / (dt + 1.0 / (2.0 * np.pi * self.cutoff_freq))
        
        # Update filter
        input_array = input_value if isinstance(input_value, np.ndarray) else np.array([input_value])
        output = alpha * input_array + (1.0 - alpha) * self.prev_output
        
        # Store output for next iteration
        self.prev_output = output
        
        return output

class DisturbanceObserver:
    """
    Disturbance Observer (DOB) for estimating and compensating external disturbances
    """
    def __init__(self, Q_filter_coeff, nominal_mass=0.088):
        """
        Initialize Disturbance Observer
        
        Args:
            Q_filter_coeff: Q-filter coefficient (determines bandwidth of disturbance estimation)
            nominal_mass: Nominal mass of the UAV (kg)
        """
        self.Q = Q_filter_coeff  # Q-filter coefficient
        self.m_nominal = nominal_mass  # Nominal mass
        
        # State variables
        self.estimated_disturbance = np.zeros(3)  # Estimated disturbance force
        self.prev_velocity = np.zeros(3)  # Previous velocity
        self.prev_control_input = np.zeros(3)  # Previous control input
        self.z = np.zeros(3)  # Internal state of the observer
        
        # Low-pass filter for smoothing disturbance estimates
        self.lpf = LowPassFilter(cutoff_freq=0.5, initial_value=np.zeros(3))
    
    def reset(self):
        """Reset observer state"""
        self.estimated_disturbance = np.zeros(3)
        self.prev_velocity = np.zeros(3)
        self.prev_control_input = np.zeros(3)
        self.z = np.zeros(3)
        self.lpf = LowPassFilter(cutoff_freq=0.5, initial_value=np.zeros(3))
    
    def update(self, velocity, control_input, dt):
        """
        Update disturbance estimation
        
        Args:
            velocity: Current velocity vector [vx, vy, vz]
            control_input: Control input vector [ux, uy, uz]
            dt: Time step in seconds
            
        Returns:
            Estimated disturbance force vector [dx, dy, dz]
        """
        # Calculate acceleration from velocity change
        acceleration = (velocity - self.prev_velocity) / dt if dt > 0 else np.zeros(3)
        
        # Update internal state z
        # z = z + Q * (m*a - u - d_hat) * dt
        nominal_dynamics = self.m_nominal * acceleration
        self.z = self.z + self.Q * (nominal_dynamics - control_input - self.estimated_disturbance) * dt
        
        # Update disturbance estimate
        # d_hat = Q * (z + u)
        raw_disturbance = self.Q * (self.z + control_input)
        
        # Apply low-pass filter to smooth disturbance estimates
        self.estimated_disturbance = self.lpf.update(raw_disturbance, dt)
        
        # Store current values for next iteration
        self.prev_velocity = velocity.copy()
        self.prev_control_input = control_input.copy()
        
        return self.estimated_disturbance

def controller(state, target_pos, dt):
    """
    Stabilized Disturbance Observer-based Controller for UAV position stabilization
    
    This controller combines a cascaded PID structure with a disturbance observer
    to estimate and compensate for external disturbances, with specific improvements
    to eliminate oscillations.
    
    Args:
        state: [position_x (m), position_y (m), position_z (m), roll (radians), pitch (radians), yaw (radians)]
        target_pos: (x (m), y (m), z (m), yaw (radians))
        dt: time step (s)
    
    Returns:
        velocity command: (velocity_x_setpoint (m/s), velocity_y_setpoint (m/s), velocity_z_setpoint (m/s), yaw_rate_setpoint (radians/s))
    """
    # Extract current state
    current_pos = np.array(state[:3])  # x, y, z position
    current_roll = state[3]  # roll angle
    current_pitch = state[4]  # pitch angle
    current_yaw = state[5]  # yaw angle
    
    # Extract target state
    target_pos_xyz = np.array(target_pos[:3])  # x, y, z position
    target_yaw = target_pos[3]  # target yaw
    
    # Initialize controllers and observers (if they don't exist)
    if not hasattr(controller, 'initialized'):

        # Position controllers (outer loop)
        controller.pos_x_controller = PIDController(1.5, 0.19, 0.3, 1.0, output_limits=(-0.8, 0.8))
        controller.pos_y_controller = PIDController(1.5, 0.19, 0.3, 1.0, output_limits=(-0.8, 0.8))
        # CRITICAL FIX: Reduced gains for z-axis to prevent oscillation
        controller.pos_z_controller = PIDController(0.5, 0.09, 0.3, 0.8, output_limits=(-0.5, 0.5))
        
        # Velocity controllers (inner loop)
        controller.vel_x_controller = PIDController(0.4, 0.05, 0.1, 0.3, output_limits=(-0.6, 0.6))
        controller.vel_y_controller = PIDController(0.4, 0.05, 0.1, 0.3, output_limits=(-0.6, 0.6))
        # CRITICAL FIX: Reduced gains for z-axis velocity controller
        controller.vel_z_controller = PIDController(0.4, 0.02, 0.1, 0.2, output_limits=(-0.4, 0.4))
        
        # Yaw controller
        controller.yaw_controller = PIDController(0.5, 0.0, 0.0, 0.1, output_limits=(-0.5, 0.5))
        
        # CRITICAL FIX: Reduced Q-filter coefficient for disturbance observer
        controller.disturbance_observer = DisturbanceObserver(Q_filter_coeff=0.3)
        
        # Store previous values for derivative calculations
        controller.prev_pos = current_pos.copy()
        controller.current_vel = np.zeros(3)
        
        # Store estimated disturbance for visualization
        controller.estimated_disturbance = np.zeros(3)
        
        # CRITICAL FIX: Add low-pass filter for z-axis commands
        controller.z_command_filter = LowPassFilter(cutoff_freq=0.7, initial_value=0.0)
        
        
        
        
        # CRITICAL FIX: Add low-pass filter for z-axis commands
        controller.x_command_filter = LowPassFilter(cutoff_freq=0.7, initial_value=0.0)
        
        
    
        # CRITICAL FIX: Add low-pass filter for z-axis commands
        controller.y_command_filter = LowPassFilter(cutoff_freq=0.7, initial_value=0.0)
        
        # CRITICAL FIX: Add deadband for z-axis to prevent small oscillations
        controller.z_deadband = 0.03

                # CRITICAL FIX: Add deadband for z-axis to prevent small oscillations
        controller.x_deadband = 0.03

                # CRITICAL FIX: Add deadband for z-axis to prevent small oscillations
        controller.y_deadband = 0.03
        
        # CRITICAL FIX: Add command history for rate limiting
        controller.prev_cmd = np.zeros(4)
        controller.max_cmd_change = np.array([0.2, 0.2, 0.1, 0.1])  # Max change per timestep
        
        controller.initialized = True
        controller.target_reached = False
    
    # Calculate position error in global frame
    pos_error_global = target_pos_xyz - current_pos
    
    
    # Check if target is reached
    
    controller.target_reached = np.linalg.norm(pos_error_global) <= 0.1  # Within 10cm threshold

    # CRITICAL FIX: Apply deadband to z-axis error to prevent small oscillations
    if abs(pos_error_global[2]) < controller.z_deadband:
        pos_error_global[2] = 0.0
    if abs(pos_error_global[0]) < controller.x_deadband:
        pos_error_global[0] = 0.0
    if abs(pos_error_global[1]) < controller.y_deadband:
        pos_error_global[1] = 0.0
    # Outer loop: Position Control



    # Calculate velocity setpoints from position errors (in global frame)
    vel_setpoint_x = controller.pos_x_controller.update(pos_error_global[0], dt)
    vel_setpoint_y = controller.pos_y_controller.update(pos_error_global[1], dt)
    vel_setpoint_z = controller.pos_z_controller.update(pos_error_global[2], dt)
    feedforward_gain = 0.2  # Tune empirically
    vel_setpoint_x += feedforward_gain * pos_error_global[0]
    vel_setpoint_y += feedforward_gain * pos_error_global[1]
    vel_setpoint_z += feedforward_gain * pos_error_global[2]
    # Estimate current velocity from position changes
    controller.current_vel = (current_pos - controller.prev_pos) / dt if dt > 0 else np.zeros(3)
    
    # Update disturbance observer
    control_input = np.array([vel_setpoint_x, vel_setpoint_y, vel_setpoint_z])
    estimated_disturbance = controller.disturbance_observer.update(controller.current_vel, control_input, dt)
    controller.estimated_disturbance = estimated_disturbance  # Store for visualization
    
    # CRITICAL FIX: Scale down disturbance compensation for z-axis
    disturbance_compensation = estimated_disturbance.copy()
    disturbance_compensation[2] *= 1.0  # Reduce z-axis disturbance compensation
    
    # Compensate for disturbance in velocity setpoints
    vel_setpoint_x += disturbance_compensation[0]
    vel_setpoint_y += disturbance_compensation[1]
    vel_setpoint_z += disturbance_compensation[2]
    
    # Inner loop: Velocity Control
    # Calculate velocity errors
    vel_error_x = vel_setpoint_x - controller.current_vel[0]
    vel_error_y = vel_setpoint_y - controller.current_vel[1]
    vel_error_z = vel_setpoint_z - controller.current_vel[2]
    
    # Calculate acceleration commands
    acc_cmd_x = controller.vel_x_controller.update(vel_error_x, dt)
    acc_cmd_y = controller.vel_y_controller.update(vel_error_y, dt)
    acc_cmd_z = controller.vel_z_controller.update(vel_error_z, dt)
    
    # Transform acceleration commands from global to body frame
    cos_yaw = cos(current_yaw)
    sin_yaw = sin(current_yaw)
    
    # Rotation matrix for global to body transformation
    R = np.array([
        [cos_yaw, sin_yaw, 0],
        [-sin_yaw, cos_yaw, 0],
        [0, 0, 1]
    ])
    
    # Transform acceleration command to body frame
    acc_cmd_body = R @ np.array([acc_cmd_x, acc_cmd_y, acc_cmd_z])
    
    # Calculate yaw error and normalize to [-pi, pi]
    yaw_error = target_yaw - current_yaw
    while yaw_error > np.pi:
        yaw_error -= 2 * np.pi
    while yaw_error < -np.pi:
        yaw_error += 2 * np.pi
    
    # Calculate yaw rate command
    yaw_rate_cmd = controller.yaw_controller.update(yaw_error, dt)
    
    # Update previous position for next iteration
    controller.prev_pos = current_pos.copy()
    
    # Apply gain multiplier to ensure commands are strong enough
    gain_multiplier_xy = 2.0  # Reduced from 3.0
    gain_multiplier_z = 1.0   # Reduced from 2.0
    acc_cmd_body[0] *= gain_multiplier_xy
    acc_cmd_body[1] *= gain_multiplier_xy
    acc_cmd_body[2] *= gain_multiplier_z
    
    # CRITICAL FIX: Apply low-pass filter to z-axis command to smooth it
    acc_cmd_body[2] = controller.z_command_filter.update(acc_cmd_body[2], dt)[0]
    
    # CRITICAL FIX: Implement gradual z-axis control instead of forcing direction
    # Only apply gentle correction when far from target
    if abs(pos_error_global[2]) > 0.05:
        if current_pos[2] > target_pos_xyz[2]:
            acc_cmd_body[2] = min(acc_cmd_body[2], -0.3)
        else:
            acc_cmd_body[2] = max(acc_cmd_body[2], 0.3)
    
    # Add a minimum command threshold for x and y to overcome inertia
    min_threshold = 0.2
    if abs(pos_error_global[0]) > 0.01:  # Only apply if error is significant
        if abs(acc_cmd_body[0]) < min_threshold and abs(pos_error_global[0]) > 0.005:
            acc_cmd_body[0] = min_threshold * np.sign(acc_cmd_body[0])
    
    if abs(pos_error_global[1]) > 0.01:  # Only apply if error is significant
        if abs(acc_cmd_body[1]) < min_threshold and abs(pos_error_global[1]) > 0.005:
            acc_cmd_body[1] = min_threshold * np.sign(acc_cmd_body[1])
    
    # Ensure commands are within limits
    acc_cmd_body[0] = np.clip(acc_cmd_body[0], -1.0, 1.0)
    acc_cmd_body[1] = np.clip(acc_cmd_body[1], -1.0, 1.0)
    acc_cmd_body[2] = np.clip(acc_cmd_body[2], -1.0, 1.0)
    yaw_rate_cmd = np.clip(yaw_rate_cmd, -1.0, 1.0)
    
    # CRITICAL FIX: Rate limit the commands to prevent sudden changes
    raw_cmd = np.array([acc_cmd_body[0], acc_cmd_body[1], acc_cmd_body[2], yaw_rate_cmd])
    cmd_change = raw_cmd - controller.prev_cmd
    limited_change = np.clip(cmd_change, -controller.max_cmd_change, controller.max_cmd_change)
    limited_cmd = controller.prev_cmd + limited_change
    controller.prev_cmd = limited_cmd
    
    # Print debug info occasionally
    if not hasattr(controller, 'debug_counter'):
        controller.debug_counter = 0
    
    controller.debug_counter += 1
    if controller.debug_counter >= 50:  # Print debug info every 50 calls
        controller.debug_counter = 0
        print("\n--- Stabilized Disturbance Observer-based Controller Debug Info ---")
        print(f"Current position: {current_pos}")
        print(f"Target position: {target_pos_xyz}")
        print(f"Position error (global): {pos_error_global}")
        print(f"Velocity setpoint (before DOB): {[vel_setpoint_x - disturbance_compensation[0], vel_setpoint_y - disturbance_compensation[1], vel_setpoint_z - disturbance_compensation[2]]}")
        print(f"Estimated disturbance: {estimated_disturbance}")
        print(f"Disturbance compensation (scaled): {disturbance_compensation}")
        print(f"Velocity setpoint (after DOB): {[vel_setpoint_x, vel_setpoint_y, vel_setpoint_z]}")
        print(f"Current velocity: {controller.current_vel}")
        print(f"Raw commands: {raw_cmd}")
        print(f"Rate-limited commands: {limited_cmd}")
        print(f"Yaw error: {yaw_error:.2f}, Yaw rate command: {yaw_rate_cmd:.2f}")
        print(f"Target hit: {controller.target_reached}")
        print("-------------------------------------------\n")
    
    # Return velocity command in the format expected by the simulator
    output = (float(limited_cmd[0]), float(limited_cmd[1]), float(limited_cmd[2]), float(limited_cmd[3]))
    return output
