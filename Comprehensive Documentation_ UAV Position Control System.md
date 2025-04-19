# Comprehensive Documentation: UAV Position Control System

## Table of Contents
1. [Introduction](#introduction)
2. [Control Theory Background](#control-theory-background)
3. [Controller Evolution](#controller-evolution)
   - [Basic PID Controller](#basic-pid-controller)
   - [Cascaded Controller](#cascaded-controller)
   - [Multi-Layer Cascaded Controller](#multi-layer-cascaded-controller)
   - [Fixed Coordinate Transformation](#fixed-coordinate-transformation)
   - [Disturbance Observer-based Controller](#disturbance-observer-based-controller)
   - [Stabilized Disturbance Observer Controller](#stabilized-disturbance-observer-controller)
   - [Direct Offset Compensation Controller](#direct-offset-compensation-controller)
4. [Detailed Code Explanation](#detailed-code-explanation)
   - [PID Controller Class](#pid-controller-class)
   - [Low-Pass Filter Class](#low-pass-filter-class)
   - [Disturbance Observer Class](#disturbance-observer-class)
   - [Main Controller Function](#main-controller-function)
   - [Simulator and Visualization](#simulator-and-visualization)
5. [Testing and Tuning Methodology](#testing-and-tuning-methodology)
6. [Troubleshooting Common Issues](#troubleshooting-common-issues)
7. [Conclusion](#conclusion)

## Introduction

This document provides a comprehensive explanation of the UAV position control system developed for the AERO60492 Autonomous Mobile Robots coursework. The project involved creating a feedback control algorithm for position stabilization of a UAV (drone), implementing it in Python, and testing it in a simulator.

The control system evolved through multiple iterations, starting from a basic PID controller and progressing to more sophisticated implementations including cascaded controllers, multi-layer cascaded controllers, and disturbance observer-based controllers. Each iteration addressed specific challenges encountered during testing, such as coordinate transformation issues, oscillations, and position offset problems.

## Control Theory Background

### Feedback Control Systems

A feedback control system uses measurements of the system's output to adjust the control input, aiming to achieve a desired output (setpoint). The controller calculates the error between the current state and the desired state, then generates appropriate commands to minimize this error.

### PID Control

PID (Proportional-Integral-Derivative) control is a widely used feedback control mechanism that calculates an error value as the difference between a desired setpoint and a measured process variable, then applies a correction based on proportional, integral, and derivative terms:

- **Proportional (P)**: Produces an output proportional to the current error. It provides an immediate response to the current error but may not eliminate steady-state error.
- **Integral (I)**: Accumulates the error over time. It helps eliminate steady-state error but can cause overshoot.
- **Derivative (D)**: Calculates the rate of change of the error. It provides damping to reduce overshoot but is sensitive to noise.

The PID controller output is calculated as:

```
output = Kp * error + Ki * ∫error dt + Kd * d(error)/dt
```

Where:
- `Kp`, `Ki`, and `Kd` are the proportional, integral, and derivative gains
- `error` is the difference between the setpoint and the measured value

### Cascaded Control

Cascaded control involves multiple control loops arranged in a hierarchical structure, where the output of one controller becomes the setpoint for the next controller. For UAV control, this typically involves:

1. **Position Controller (Outer Loop)**: Takes position errors and generates velocity setpoints
2. **Velocity Controller (Middle Loop)**: Takes velocity errors and generates acceleration setpoints
3. **Acceleration Controller (Inner Loop)**: Takes acceleration errors and generates attitude setpoints
4. **Attitude Controller (Innermost Loop)**: Takes attitude errors and generates rate commands

This structure provides better performance by allowing each loop to handle disturbances at its own level and operate at appropriate frequencies.

### Disturbance Observer

A Disturbance Observer (DOB) is an advanced control technique that estimates external disturbances acting on the system and compensates for them in the control commands. It improves robustness against model uncertainties and external forces by:

1. Estimating the difference between expected and actual dynamics
2. Attributing this difference to external disturbances
3. Compensating for these disturbances in the control commands

## Controller Evolution

### Basic PID Controller

The initial implementation was a basic PID controller that took position errors as input and generated velocity commands as output. This controller included:

- Separate PID controllers for x, y, z positions and yaw angle
- Coordinate transformation from global to body frame
- Anti-windup protection to prevent integral term saturation

```python
# Example of basic PID controller structure
def controller(state, target_pos, dt):
    # Extract current state and target
    current_pos = np.array(state[:3])
    current_yaw = state[5]
    target_pos_xyz = np.array(target_pos[:3])
    target_yaw = target_pos[3]
    
    # Calculate position error in global frame
    pos_error_global = target_pos_xyz - current_pos
    
    # Calculate velocity setpoints using PID controllers
    vel_setpoint_x = pos_x_controller.update(pos_error_global[0], dt)
    vel_setpoint_y = pos_y_controller.update(pos_error_global[1], dt)
    vel_setpoint_z = pos_z_controller.update(pos_error_global[2], dt)
    
    # Transform velocity commands from global to body frame
    cos_yaw = cos(current_yaw)
    sin_yaw = sin(current_yaw)
    R = np.array([
        [cos_yaw, sin_yaw, 0],
        [-sin_yaw, cos_yaw, 0],
        [0, 0, 1]
    ])
    vel_cmd_body = R @ np.array([vel_setpoint_x, vel_setpoint_y, vel_setpoint_z])
    
    # Calculate yaw rate command
    yaw_error = target_yaw - current_yaw
    yaw_rate_cmd = yaw_controller.update(yaw_error, dt)
    
    return (vel_cmd_body[0], vel_cmd_body[1], vel_cmd_body[2], yaw_rate_cmd)
```

This basic controller worked for simple scenarios but had limitations in handling complex dynamics and disturbances.

### Cascaded Controller

To improve performance, a cascaded controller was implemented with two control loops:

1. **Outer Loop (Position Control)**: Takes position errors and generates velocity setpoints
2. **Inner Loop (Velocity Control)**: Takes velocity errors and generates acceleration commands

```python
# Example of cascaded controller structure
def controller(state, target_pos, dt):
    # ... (state extraction) ...
    
    # Outer loop: Position Control
    vel_setpoint_x = pos_x_controller.update(pos_error_global[0], dt)
    vel_setpoint_y = pos_y_controller.update(pos_error_global[1], dt)
    vel_setpoint_z = pos_z_controller.update(pos_error_global[2], dt)
    
    # Estimate current velocity
    current_vel = (current_pos - prev_pos) / dt
    
    # Inner loop: Velocity Control
    vel_error_x = vel_setpoint_x - current_vel[0]
    vel_error_y = vel_setpoint_y - current_vel[1]
    vel_error_z = vel_setpoint_z - current_vel[2]
    
    acc_cmd_x = vel_x_controller.update(vel_error_x, dt)
    acc_cmd_y = vel_y_controller.update(vel_error_y, dt)
    acc_cmd_z = vel_z_controller.update(vel_error_z, dt)
    
    # Transform to body frame and return
    # ...
```

This cascaded structure provided better control over the drone's dynamics by separating the position and velocity control problems.

### Multi-Layer Cascaded Controller

To further improve control performance, a multi-layer cascaded controller was implemented with four control loops:

1. **Position Controller (Outer Loop)**: Takes position errors and generates velocity setpoints
2. **Velocity Controller**: Takes velocity errors and generates acceleration setpoints
3. **Acceleration Controller**: Takes acceleration errors and generates attitude setpoints
4. **Attitude Controller (Inner Loop)**: Takes attitude errors and generates rate commands

```python
# Example of multi-layer cascaded controller structure
def controller(state, target_pos, dt):
    # ... (state extraction) ...
    
    # Layer 1: Position Control
    vel_setpoint_x = pos_x_controller.update(pos_error_global[0], dt)
    vel_setpoint_y = pos_y_controller.update(pos_error_global[1], dt)
    vel_setpoint_z = pos_z_controller.update(pos_error_global[2], dt)
    
    # Layer 2: Velocity Control
    vel_error_x = vel_setpoint_x - current_vel[0]
    vel_error_y = vel_setpoint_y - current_vel[1]
    vel_error_z = vel_setpoint_z - current_vel[2]
    
    acc_setpoint_x = vel_x_controller.update(vel_error_x, dt)
    acc_setpoint_y = vel_y_controller.update(vel_error_y, dt)
    acc_setpoint_z = vel_z_controller.update(vel_error_z, dt)
    
    # Layer 3: Acceleration Control
    acc_error_x = acc_setpoint_x - current_acc[0]
    acc_error_y = acc_setpoint_y - current_acc[1]
    acc_error_z = acc_setpoint_z - current_acc[2]
    
    att_setpoint_x = acc_x_controller.update(acc_error_x, dt)
    att_setpoint_y = acc_y_controller.update(acc_error_y, dt)
    att_setpoint_z = acc_z_controller.update(acc_error_z, dt)
    
    # Layer 4: Attitude Control
    # ...
```

This multi-layer structure provided even finer control over the drone's behavior but introduced complexity in tuning and coordination between the layers.

### Fixed Coordinate Transformation

During testing, it was discovered that the drone wasn't moving correctly toward the target due to an issue with the coordinate transformation. The transformation matrix was corrected:

```python
# Incorrect transformation matrix
R = np.array([
    [cos_yaw, -sin_yaw, 0],
    [sin_yaw, cos_yaw, 0],
    [0, 0, 1]
])

# Corrected transformation matrix
R = np.array([
    [cos_yaw, sin_yaw, 0],
    [-sin_yaw, cos_yaw, 0],
    [0, 0, 1]
])
```

This correction ensured that the velocity commands were properly transformed from the global frame to the body frame, allowing the drone to move correctly toward the target.

### Disturbance Observer-based Controller

To improve robustness against external disturbances and model uncertainties, a Disturbance Observer-based controller was implemented. This controller estimated external disturbances acting on the drone and compensated for them in the control commands.

```python
# Example of Disturbance Observer update
def update(self, velocity, control_input, dt):
    # Calculate acceleration from velocity change
    acceleration = (velocity - self.prev_velocity) / dt
    
    # Update internal state z
    nominal_dynamics = self.m_nominal * acceleration
    self.z = self.z + self.Q * (nominal_dynamics - control_input - self.estimated_disturbance) * dt
    
    # Update disturbance estimate
    raw_disturbance = self.Q * (self.z + control_input)
    self.estimated_disturbance = self.lpf.update(raw_disturbance, dt)
    
    # Store current values for next iteration
    self.prev_velocity = velocity.copy()
    self.prev_control_input = control_input.copy()
    
    return self.estimated_disturbance
```

The disturbance observer was integrated into the cascaded control structure:

```python
# Example of Disturbance Observer integration
def controller(state, target_pos, dt):
    # ... (position and velocity control) ...
    
    # Update disturbance observer
    control_input = np.array([vel_setpoint_x, vel_setpoint_y, vel_setpoint_z])
    estimated_disturbance = disturbance_observer.update(current_vel, control_input, dt)
    
    # Compensate for disturbance in velocity setpoints
    vel_setpoint_x += estimated_disturbance[0]
    vel_setpoint_y += estimated_disturbance[1]
    vel_setpoint_z += estimated_disturbance[2]
    
    # ... (continue with inner loop control) ...
```

This controller provided better performance in the presence of external disturbances but introduced oscillations in some scenarios.

### Stabilized Disturbance Observer Controller

To address the oscillation issues observed with the Disturbance Observer-based controller, a Stabilized Disturbance Observer Controller was implemented with several critical fixes:

1. **Reduced Gains for Z-Axis**: Lower gains for the z-axis to prevent oscillation
2. **Low-Pass Filtering**: Added low-pass filters to smooth control commands
3. **Deadband Implementation**: Added deadbands to ignore small errors
4. **Rate Limiting**: Limited the rate of change of control commands
5. **Gradual Z-Axis Control**: Implemented gentler z-axis control

```python
# Example of stabilization features
def controller(state, target_pos, dt):
    # ... (state extraction) ...
    
    # CRITICAL FIX: Apply deadband to z-axis error to prevent small oscillations
    if abs(pos_error_global[2]) < controller.z_deadband:
        pos_error_global[2] = 0.0
    
    # ... (position and velocity control) ...
    
    # CRITICAL FIX: Apply low-pass filter to z-axis command to smooth it
    acc_cmd_body[2] = controller.z_command_filter.update(acc_cmd_body[2], dt)[0]
    
    # CRITICAL FIX: Implement gradual z-axis control instead of forcing direction
    if abs(pos_error_global[2]) > 0.05:
        if current_pos[2] > target_pos_xyz[2]:
            acc_cmd_body[2] = min(acc_cmd_body[2], -0.3)
        else:
            acc_cmd_body[2] = max(acc_cmd_body[2], 0.3)
    
    # CRITICAL FIX: Rate limit the commands to prevent sudden changes
    raw_cmd = np.array([acc_cmd_body[0], acc_cmd_body[1], acc_cmd_body[2], yaw_rate_cmd])
    cmd_change = raw_cmd - controller.prev_cmd
    limited_change = np.clip(cmd_change, -controller.max_cmd_change, controller.max_cmd_change)
    limited_cmd = controller.prev_cmd + limited_change
    controller.prev_cmd = limited_cmd
```

These stabilization features significantly reduced oscillations while maintaining the disturbance rejection capabilities of the controller.

### Direct Offset Compensation Controller

Despite the improvements in the previous controllers, there was still a persistent offset between the drone's position and the target position. To address this, a Direct Offset Compensation Controller was implemented that directly added a fixed offset to the target position:

```python
# Example of Direct Offset Compensation
def controller(state, target_pos, dt):
    # ... (state extraction) ...
    
    # DIRECT OFFSET COMPENSATION: Apply offset compensation to target position
    compensated_target = target_pos_xyz + controller.offset_compensation
    
    # Calculate position error using compensated target
    pos_error_global = compensated_target - current_pos
    
    # ... (continue with control) ...
```

The controller also included an adaptive learning mechanism to automatically adjust the offset compensation based on the drone's actual position relative to the target:

```python
# Example of adaptive offset learning
if controller.learning_enabled and len(controller.position_history) > 10:
    # Calculate average position during hold
    avg_pos = np.mean(np.array(controller.position_history[-10:]), axis=0)
    # Calculate error between average position and target
    actual_error = target_pos_xyz - avg_pos
    # Update offset compensation
    controller.offset_compensation += controller.learning_rate * actual_error
```

This approach provided a direct solution to the offset issue by explicitly compensating for the observed offset.

## Detailed Code Explanation

### PID Controller Class

The `PIDController` class implements a PID controller with anti-windup protection:

```python
class PIDController:
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
```

Key components:
- **Initialization**: Sets up the controller with specified gains and limits
- **Reset Method**: Clears the controller state (integral term and previous error)
- **Update Method**: Calculates the control output based on the current error
  - Updates the integral term with anti-windup protection
  - Calculates the derivative term
  - Combines the P, I, and D terms to produce the output
  - Applies output limits if specified

The anti-windup protection is implemented by clipping the integral term to prevent it from growing too large when the controller output is saturated.

### Low-Pass Filter Class

The `LowPassFilter` class implements a first-order low-pass filter for smoothing signals:

```python
class LowPassFilter:
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
```

Key components:
- **Initialization**: Sets up the filter with a specified cutoff frequency and initial value
- **Update Method**: Applies the filter to a new input value
  - Calculates the filter coefficient based on the cutoff frequency and time step
  - Updates the filter output as a weighted average of the input and previous output
  - Stores the output for the next iteration

The filter smooths signals by reducing high-frequency components, which helps reduce noise and prevent rapid changes in control commands.

### Disturbance Observer Class

The `DisturbanceObserver` class implements a disturbance observer for estimating and compensating external disturbances:

```python
class DisturbanceObserver:
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
```

Key components:
- **Initialization**: Sets up the observer with a specified Q-filter coefficient and nominal mass
- **Reset Method**: Clears the observer state
- **Update Method**: Estimates the disturbance based on the difference between expected and actual dynamics
  - Calculates acceleration from velocity change
  - Updates the internal state based on the nominal dynamics, control input, and estimated disturbance
  - Updates the disturbance estimate based on the internal state and control input
  - Applies a low-pass filter to smooth the disturbance estimates
  - Stores current values for the next iteration

The disturbance observer works by comparing the expected dynamics (based on the nominal mass and measured acceleration) with the actual control input and attributing the difference to external disturbances.

### Main Controller Function

The main `controller` function implements the control algorithm for UAV position stabilization. Here's a detailed explanation of the stabilized disturbance observer controller:

```python
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
        controller.x_command_filter = LowPassFilter(cutoff_freq=0.7, initial_value=0.0)
        controller.y_command_filter = LowPassFilter(cutoff_freq=0.7, initial_value=0.0)
        
        # CRITICAL FIX: Add deadband for z-axis to prevent small oscillations
        controller.z_deadband = 0.03
        controller.x_deadband = 0.03
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
    
    # Add feedforward term for faster response
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
        print(f"Disturbance-compensated velocity setpoint: {[vel_setpoint_x, vel_setpoint_y, vel_setpoint_z]}")
        print(f"Current velocity: {controller.current_vel}")
        print(f"Velocity error: {[vel_error_x, vel_error_y, vel_error_z]}")
        print(f"Acceleration command (global): {[acc_cmd_x, acc_cmd_y, acc_cmd_z]}")
        print(f"Acceleration command (body): {acc_cmd_body}")
        print(f"Yaw error: {yaw_error}")
        print(f"Yaw rate command: {yaw_rate_cmd}")
        print(f"Target reached: {controller.target_reached}")
        print("-------------------------------------------\n")
    
    # Return velocity command in the format expected by the simulator
    return (float(limited_cmd[0]), float(limited_cmd[1]), float(limited_cmd[2]), float(limited_cmd[3]))
```

Key components:
1. **State Extraction**: Extracts the current position, orientation, and target position from the input parameters
2. **Controller Initialization**: Sets up the PID controllers, disturbance observer, and other components if they don't exist
3. **Position Error Calculation**: Calculates the error between the current position and the target position
4. **Deadband Application**: Applies deadbands to ignore small errors and prevent oscillations
5. **Outer Loop (Position Control)**: Generates velocity setpoints based on position errors
6. **Disturbance Observer Update**: Estimates external disturbances based on the difference between expected and actual dynamics
7. **Disturbance Compensation**: Adds the estimated disturbances to the velocity setpoints
8. **Inner Loop (Velocity Control)**: Generates acceleration commands based on velocity errors
9. **Coordinate Transformation**: Transforms the acceleration commands from the global frame to the body frame
10. **Command Smoothing**: Applies low-pass filtering to smooth the commands
11. **Gradual Z-Axis Control**: Implements gentler z-axis control to prevent oscillations
12. **Minimum Command Threshold**: Ensures commands are strong enough to overcome inertia
13. **Command Limiting**: Ensures commands are within acceptable limits
14. **Rate Limiting**: Limits the rate of change of commands to prevent sudden changes
15. **Debug Information**: Prints debug information periodically

The controller combines a cascaded PID structure with a disturbance observer and several stabilization features to achieve robust and stable position control.

### Simulator and Visualization

The simulator script (`stabilized_disturbance_observer_run.py`) provides a simulation environment for testing the controller and visualizing its behavior. Here's a detailed explanation of its key components:

```python
class Simulator:
    def __init__(self):
        # Initialize PyBullet simulation
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load plane and drone
        self.plane_id = p.loadURDF("plane.urdf")
        self.start_pos = [0, 0, 1]
        self.start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.drone_id = p.loadURDF(
            "resources/tello.urdf", self.start_pos, self.start_orientation
        )
        
        # Set up drone dynamics parameters
        # ...
        
        # Load targets
        self.targets = self.load_targets()
        self.current_target = 0
        
        # Create target marker
        # ...
        
        # Set up data logging
        self.log_data = []
        self.is_logging = False
        
        # Create visualization markers
        self.create_visualizers()
        
        # Create visualization lines
        self.error_line_id = -1
        self.cmd_line_id = -1
        self.disturbance_line_id = -1
        self.deadband_upper_id = -1
        self.deadband_lower_id = -1
```

The simulator initializes the PyBullet simulation environment, loads the drone and plane models, sets up the drone dynamics parameters, loads the target positions, and creates visualization markers.

```python
def create_visualizers(self):
    """Create visualization markers"""
    # Velocity setpoint marker
    vel_visual_id = p.createVisualShape(
        shapeType=p.GEOM_SPHERE, radius=0.03, rgbaColor=[0, 1, 0, 0.5]
    )
    self.vel_marker_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=vel_visual_id,
        basePosition=self.start_pos,
    )
    
    # Disturbance estimation marker
    dist_visual_id = p.createVisualShape(
        shapeType=p.GEOM_SPHERE, radius=0.03, rgbaColor=[1, 0, 0, 0.5]
    )
    self.dist_marker_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=dist_visual_id,
        basePosition=self.start_pos,
    )
    
    # Compensated velocity marker
    comp_visual_id = p.createVisualShape(
        shapeType=p.GEOM_SPHERE, radius=0.03, rgbaColor=[0, 0, 1, 0.5]
    )
    self.comp_marker_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=comp_visual_id,
        basePosition=self.start_pos,
    )
```

The `create_visualizers` method creates visualization markers for the velocity setpoint, disturbance estimation, and compensated velocity.

```python
def update_visualizers(self, pos, yaw, target_pos):
    """Update visualization markers"""
    if not hasattr(controller.controller, 'estimated_disturbance'):
        return
        
    # Get current position and estimated disturbance
    current_pos = np.array(pos)
    estimated_disturbance = controller.controller.estimated_disturbance
    
    # Scale factors for visualization
    scale = 1.0
    
    # Create rotation matrix from yaw for visualization (body to global)
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    R = np.array([
        [cos_yaw, -sin_yaw, 0],
        [sin_yaw, cos_yaw, 0],
        [0, 0, 1]
    ])
    
    # Update marker positions
    # ...
    
    # Update disturbance visualization line
    if self.disturbance_line_id != -1:
        p.removeUserDebugItem(self.disturbance_line_id)
    
    # Transform disturbance to global frame for visualization
    dist_global = R @ estimated_disturbance
    
    # Draw a line representing the estimated disturbance
    dist_end = dist_pos + dist_global * scale * 2.0  # Amplify for visibility
    self.disturbance_line_id = p.addUserDebugLine(
        lineFromXYZ=dist_pos,
        lineToXYZ=dist_end,
        lineColorRGB=[1, 0, 0],  # Red
        lineWidth=2.0,
        lifeTime=0  # Persistent until removed
    )
    
    # Update deadband visualization
    # ...
```

The `update_visualizers` method updates the visualization markers and lines based on the current state of the controller.

```python
def update_error_visualization(self, pos, target_pos):
    """Update the line visualizing the error vector"""
    # Remove previous line if it exists
    if self.error_line_id != -1:
        p.removeUserDebugItem(self.error_line_id)
        
    # Draw a new line from current position to target
    self.error_line_id = p.addUserDebugLine(
        lineFromXYZ=pos,
        lineToXYZ=target_pos[:3],
        lineColorRGB=[1, 0, 0],  # Red
        lineWidth=2.0,
        lifeTime=0  # Persistent until removed
    )
```

The `update_error_visualization` method updates the line visualizing the error vector between the current position and the target position.

```python
def update_command_visualization(self, pos, cmd, yaw):
    """Update the line visualizing the velocity command"""
    # Remove previous line if it exists
    if self.cmd_line_id != -1:
        p.removeUserDebugItem(self.cmd_line_id)
        
    # Create rotation matrix from yaw for visualization (body to global)
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    R = np.array([
        [cos_yaw, -sin_yaw, 0],
        [sin_yaw, cos_yaw, 0],
        [0, 0, 1]
    ])
    
    # Transform command from body to global frame for visualization
    cmd_global = R @ np.array(cmd[:3])
    
    # Scale command for better visualization
    scale = 1.0
    cmd_end = np.array(pos) + cmd_global * scale
    
    # Draw a new line from current position in the direction of the command
    self.cmd_line_id = p.addUserDebugLine(
        lineFromXYZ=pos,
        lineToXYZ=cmd_end,
        lineColorRGB=[0, 1, 0],  # Green
        lineWidth=2.0,
        lifeTime=0  # Persistent until removed
    )
```

The `update_command_visualization` method updates the line visualizing the velocity command.

```python
def apply_disturbance(self, strength=0.3):
    """Apply a random disturbance force to the drone"""
    # Generate random disturbance force
    disturbance_force = np.array([
        np.random.uniform(-strength, strength),
        np.random.uniform(-strength, strength),
        np.random.uniform(-strength/3, strength/3)  # Less vertical disturbance
    ])
    
    # Apply the disturbance force
    p.applyExternalForce(
        objectUniqueId=self.drone_id,
        linkIndex=-1,
        forceObj=disturbance_force,
        posObj=[0, 0, 0],
        flags=p.LINK_FRAME,
    )
    
    print(f"INFO: Applied random disturbance force: {disturbance_force}")
```

The `apply_disturbance` method applies a random disturbance force to the drone to test the controller's disturbance rejection capabilities.

The main simulation loop runs the following steps:
1. Get the current state of the drone (position, orientation, velocity)
2. Apply random disturbances at regular intervals
3. Run the position control loop at a specified frequency
4. Get the controller output
5. Update the visualization markers and lines
6. Log flight data
7. Compute forces and torques based on the controller output
8. Apply forces and torques to the drone
9. Handle user input (keyboard events)
10. Step the simulation

## Testing and Tuning Methodology

The controllers were tested and tuned using the following methodology:

1. **Initial Gain Selection**: Start with conservative gains based on theoretical considerations and prior experience
2. **Step Response Testing**: Apply step changes in the target position and observe the response
3. **Disturbance Testing**: Apply external disturbances and observe the controller's ability to reject them
4. **Gain Tuning**: Adjust the gains based on the observed performance
   - Increase proportional gain for faster response
   - Increase integral gain to eliminate steady-state error
   - Increase derivative gain to reduce overshoot
5. **Stability Analysis**: Ensure the controller remains stable under various conditions
6. **Fine-Tuning**: Make small adjustments to optimize performance

For the stabilized disturbance observer controller, the following specific tuning steps were taken:

1. **Reduce Z-Axis Gains**: Lower the gains for the z-axis to prevent oscillations
2. **Add Low-Pass Filtering**: Tune the cutoff frequency of the low-pass filters to smooth commands without introducing excessive delay
3. **Set Deadband Values**: Choose appropriate deadband values to ignore small errors without affecting overall performance
4. **Adjust Rate Limits**: Set rate limits to prevent sudden changes in commands while maintaining responsiveness

## Troubleshooting Common Issues

During the development of the controllers, several common issues were encountered and addressed:

1. **Coordinate Transformation Issues**:
   - **Symptom**: Drone moves in the wrong direction relative to the target
   - **Solution**: Correct the rotation matrix for global to body transformation

2. **Oscillations**:
   - **Symptom**: Drone oscillates around the target position, especially in the z-axis
   - **Solution**: Reduce gains, add low-pass filtering, implement deadbands, and limit command rates

3. **Position Offset**:
   - **Symptom**: Drone stabilizes at a position offset from the target
   - **Solution**: Increase integral gains or implement direct offset compensation

4. **Slow Response**:
   - **Symptom**: Drone takes too long to reach the target position
   - **Solution**: Increase proportional gains, add feedforward terms, or implement minimum command thresholds

5. **Disturbance Sensitivity**:
   - **Symptom**: Drone is easily disturbed by external forces
   - **Solution**: Implement a disturbance observer to estimate and compensate for external disturbances

## Conclusion

This document has provided a comprehensive explanation of the UAV position control system developed for the AERO60492 Autonomous Mobile Robots coursework. The control system evolved through multiple iterations, each addressing specific challenges encountered during testing.

The final stabilized disturbance observer controller combines a cascaded PID structure with a disturbance observer and several stabilization features to achieve robust and stable position control. It demonstrates the application of advanced control techniques to solve real-world problems in autonomous mobile robotics.

The key lessons learned from this project include:
1. The importance of proper coordinate transformations in UAV control
2. The effectiveness of cascaded control structures for complex systems
3. The benefits of disturbance observers for improving robustness
4. The need for stabilization features to prevent oscillations
5. The value of visualization tools for understanding controller behavior

These lessons can be applied to other control problems in autonomous mobile robotics, providing a foundation for developing robust and effective control systems.
