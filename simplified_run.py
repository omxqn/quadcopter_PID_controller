import pybullet as p
import time
import csv
import pybullet_data
import numpy as np
from src.tello_controller import TelloController
import importlib
import sys
import os

# Add the assignment directory to the path to import the simplified controller
sys.path.append('/home/ubuntu/assignment')
try:
    import simplified_controller as controller
    print("Using simplified controller")
except ImportError:
    # Fall back to the original controller if simplified_controller is not found
    import controller
    print("Using original controller")


class Simulator:
    def __init__(self):
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF("plane.urdf")
        self.start_pos = [0, 0, 1]
        self.start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.drone_id = p.loadURDF(
            "resources/tello.urdf", self.start_pos, self.start_orientation
        )

        # Constants for dynamics calculations from the paper
        # https://ieeexplore.ieee.org/document/9836168
        self.M = 0.088  # Mass of UAV (kg)
        self.L = 0.06  # Distance from rotor axis to center of mass (m)
        # Inertia matrix (kg*m^2)
        self.IR = 4.95e-5  # Rotor inertia (kg*m^2)
        self.KF = 0.566e-5  # Thrust constant (kg*m/rad^2) This was wildly wrong in the paper. Calculated from mass of tello and 15000rpm to hover.
        self.KM = 0.762e-7  # Reaction torque constant factor (kg*m^2/rad^2)
        # Drag coefficients
        self.K_TRANS = np.array([3.365e-2, 3.365e-2, 3.365e-2])  # Translational (kg/s)
        self.K_ROT = np.array(
            [4.609e-3, 4.609e-3, 4.609e-3]
        )  # Aerodynamic friction (kg*m^2/rad)
        self.TM = 0.0163  # Motor response time constant (s)
        self.tello_controller = TelloController(
            9.81, self.M, self.L, 0.35, self.KF, self.KM
        )

        # Load targets
        self.targets = self.load_targets()
        self.current_target = 0

        # Create a red sphere for the target
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1]
        )
        self.marker_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=-1,  # No collision shape
            baseVisualShapeIndex=visual_shape_id,
            basePosition=self.targets[self.current_target][0:3],
        )
        print(f"INFO: Target set to: {self.targets[self.current_target]}")
        
        # For data logging
        self.log_data = []
        self.is_logging = False
        self.log_counter = 0
        self.log_interval = 10  # Log every 10 steps

    def load_targets(self):
        targets = []
        with open("targets.csv", "r") as file:
            csvreader = csv.reader(file)
            header = next(csvreader)
            for row in csvreader:
                if len(row) != 4:
                    print(
                        f"WARNING: Expected 4 columns, but got {len(row)} columns for row: {row}"
                    )
                    continue
                if float(row[2]) < 0:
                    print("WARNING: Target z below the ground, not loading target")
                else:
                    targets.append(
                        (float(row[0]), float(row[1]), float(row[2]), float(row[3]))
                    )
            if targets == []:
                print(
                    "WARNING: No valid targets found in targets.csv setting target to origin"
                )
                targets.append((0.0, 0.0, 0.0, 0.0))
        return targets

    def compute_dynamics(self, rpm_values, lin_vel_world, quat):
        rotation = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)

        # Convert RPM to rad/s
        omega = rpm_values * (2 * np.pi / 60)
        omega_squared = omega**2

        # Compute forces and torques
        motor_forces = omega_squared * self.KF
        thrust = np.array([0, 0, np.sum(motor_forces)])

        # Add translational drag
        vel_body = np.dot(rotation.T, lin_vel_world)
        drag_body = -self.K_TRANS * vel_body

        force = drag_body + thrust

        # Compute torques
        z_torques = omega_squared * self.KM
        z_torque = -z_torques[0] - z_torques[1] + z_torques[2] + z_torques[3]
        x_torque = (
            -motor_forces[0] + motor_forces[1] + motor_forces[2] - motor_forces[3]
        ) * (self.L)
        y_torque = (
            -motor_forces[0] + motor_forces[1] - motor_forces[2] + motor_forces[3]
        ) * (self.L)

        torques = np.array([x_torque, y_torque, z_torque])

        return force, torques

    def display_target(self):
        p.resetBasePositionAndOrientation(
            self.marker_id,
            self.targets[self.current_target][0:3],
            self.start_orientation,
        )
        print(f"INFO: Target set to: {self.targets[self.current_target]}")
        return

    def check_action(self, unchecked_action):
        # Check if the action is a tuple or list and of length 3
        if isinstance(unchecked_action, (tuple, list)):
            if len(unchecked_action) != 4:
                print(
                    "WARNING: Controller returned an action of length "
                    + str(len(unchecked_action))
                    + ", expected 4"
                )
                checked_action = (0, 0, 0, 0)
                p.disconnect()
            else:
                # Clip to the inputs the tello accepts
                checked_action = (
                    np.clip(unchecked_action[0], -1, 1),
                    np.clip(unchecked_action[1], -1, 1),
                    np.clip(unchecked_action[2], -1, 1),
                    np.clip(unchecked_action[3], -1.74533, 1.74533),  # 100 degrees/s
                )
                # checked_action = unchecked_action

        else:
            print(
                "WARNING: Controller returned an action of type "
                + str(type(unchecked_action))
                + ", expected list or tuple"
            )
            checked_action = (0, 0, 0, 0)
            p.disconnect()

        return checked_action

    def spin_motors(self, rpm, timestep):
        for joint_index in range(4):
            # RPM to rad/s
            rad_s = rpm[joint_index] * (2.0 * np.pi / 60.0)
            current_angle = p.getJointState(self.drone_id, joint_index)[0]
            new_angle = current_angle + rad_s * timestep

            # Directly set the joint angle
            p.resetJointState(
                bodyUniqueId=self.drone_id,
                jointIndex=joint_index,
                targetValue=new_angle,
            )

    def motor_model(self, desired_rpm, current_rpm, dt):
        # First order motor model
        rpm_derivative = (desired_rpm - current_rpm) / self.TM
        real_rpm = current_rpm + rpm_derivative * dt
        return real_rpm

    def reload_controller(self):
        try:
            importlib.reload(controller)
            print("INFO: Controller module reloaded successfully")
        except Exception as e:
            print(f"ERROR: Failed to reload controller module: {e}")
            return
            
    def start_logging(self):
        """Start logging flight data"""
        self.is_logging = True
        self.log_data = []
        print("INFO: Data logging started")
        
    def stop_logging(self):
        """Stop logging and save data to CSV"""
        if not self.is_logging or len(self.log_data) == 0:
            print("INFO: No data to save")
            return
            
        self.is_logging = False
        
        # Save to CSV
        filename = f"flight_data_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            # Write header
            writer.writerow([
                'time', 
                'pos_x', 'pos_y', 'pos_z', 
                'roll', 'pitch', 'yaw',
                'vel_x', 'vel_y', 'vel_z',
                'target_x', 'target_y', 'target_z', 'target_yaw',
                'cmd_vel_x', 'cmd_vel_y', 'cmd_vel_z', 'cmd_yaw_rate'
            ])
            # Write data
            writer.writerows(self.log_data)
        
        print(f"INFO: Flight data saved to {filename}")
        
    def log_flight_data(self, time_val, pos, orientation, vel, target, cmd):
        """Log flight data for analysis"""
        if not self.is_logging:
            return
            
        self.log_counter += 1
        if self.log_counter < self.log_interval:
            return
            
        self.log_counter = 0
        
        # Extract roll, pitch, yaw
        roll, pitch, yaw = orientation
        
        # Create data row
        data_row = [
            time_val,
            pos[0], pos[1], pos[2],
            roll, pitch, yaw,
            vel[0], vel[1], vel[2],
            target[0], target[1], target[2], target[3],
            cmd[0], cmd[1], cmd[2], cmd[3]
        ]
        
        self.log_data.append(data_row)


if __name__ == "__main__":
    print("Starting UAV Position Control Simulator with Simplified Controller")
    print("Using controller:", controller.__file__)

    sim = Simulator()
    # Simulation parameters
    timestep = 1.0 / 1000  # 1000 Hz
    pos_control_timestep = 1.0 / 50  # 50 Hz
    steps_between_pos_control = int(round(pos_control_timestep / timestep))
    loop_counter = 0
    sim_time = 0.0

    prev_rpm = np.array([0, 0, 0, 0])
    desired_vel = np.array([0, 0, 0])
    yaw_rate_setpoint = 0

    # Main simulation loop
    while True:
        loop_start = time.time()
        loop_counter += 1
        sim_time += timestep

        pos, quat = p.getBasePositionAndOrientation(sim.drone_id)
        lin_vel_world, ang_vel_world = p.getBaseVelocity(sim.drone_id)

        # Extract roll, pitch, and yaw from the current orientation
        roll, pitch, yaw = p.getEulerFromQuaternion(quat)
        orientation = [roll, pitch, yaw]

        # Build a new quaternion using only yaw
        yaw_quat = p.getQuaternionFromEuler([0, 0, yaw])

        inverted_pos, inverted_quat = p.invertTransform([0, 0, 0], quat)
        inverted_pos_yaw, inverted_quat_yaw = p.invertTransform([0, 0, 0], yaw_quat)

        # Rotate the velocity vector by only yaw
        lin_vel = p.rotateVector(inverted_quat_yaw, lin_vel_world)

        ang_vel = p.rotateVector(inverted_quat, ang_vel_world)

        lin_vel = np.array(lin_vel)
        ang_vel = np.array(ang_vel)
        
        # Only run the pos control loop at given frequency
        if loop_counter >= steps_between_pos_control:
            loop_counter = 0
            # Pack the state up
            state = np.concatenate((pos, p.getEulerFromQuaternion(quat)))

            # Get controller output
            controller_output = sim.check_action(
                controller.controller(
                    state, sim.targets[sim.current_target], pos_control_timestep
                )
            )
            desired_vel = np.array(controller_output[:3])
            yaw_rate_setpoint = controller_output[3]
            
            # Log flight data
            sim.log_flight_data(
                sim_time, 
                pos, 
                orientation, 
                lin_vel, 
                sim.targets[sim.current_target], 
                controller_output
            )
            
            # Print debug info occasionally
            if int(sim_time) % 5 == 0 and int(sim_time * 10) % 10 == 0:  # Every 5 seconds
                target = sim.targets[sim.current_target]
                error = np.array(target[:3]) - np.array(pos)
                error_mag = np.linalg.norm(error)
                print(f"Time: {sim_time:.1f}s, Position: {pos}, Target: {target[:3]}, Error: {error_mag:.2f}m")
                print(f"Velocity command: {desired_vel}, Yaw rate: {yaw_rate_setpoint:.2f}")

        rpm = sim.tello_controller.compute_control(
            desired_vel, lin_vel, quat, ang_vel, yaw_rate_setpoint, timestep
        )

        rpm = sim.motor_model(rpm, prev_rpm, timestep)

        prev_rpm = rpm

        # Compute forces and torques
        force, torque = sim.compute_dynamics(rpm, lin_vel_world, quat)

        # Apply forces and torques directly
        p.applyExternalForce(
            objectUniqueId=sim.drone_id,
            linkIndex=-1,
            forceObj=force,
            posObj=[0, 0, 0],
            flags=p.LINK_FRAME,
        )
        p.applyExternalTorque(
            objectUniqueId=sim.drone_id,
            linkIndex=-1,
            torqueObj=torque,
            flags=p.LINK_FRAME,
        )

        sim.spin_motors(rpm, timestep)

        # Handle keypresses
        keys = p.getKeyboardEvents()
        if ord("r") in keys and keys[ord("r")] & p.KEY_WAS_TRIGGERED:
            p.resetBasePositionAndOrientation(
                sim.drone_id, sim.start_pos, sim.start_orientation
            )
            prev_rpm = np.array([0, 0, 0, 0])
            sim.tello_controller.reset()
            sim.reload_controller()
            sim.targets = sim.load_targets()
            sim.current_target = 0
            sim.display_target()
            print("INFO: Vehicle reset by keyboard key 'r'.")
        if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_WAS_TRIGGERED:
            sim.current_target = (sim.current_target + 1) % len(sim.targets)
            sim.display_target()
        if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_WAS_TRIGGERED:
            sim.current_target = (sim.current_target - 1) % len(sim.targets)
            sim.display_target()
        if ord("l") in keys and keys[ord("l")] & p.KEY_WAS_TRIGGERED:
            if sim.is_logging:
                sim.stop_logging()
            else:
                sim.start_logging()
        if ord("d") in keys and keys[ord("d")] & p.KEY_WAS_TRIGGERED:
            # Add a disturbance force
            p.applyExternalForce(
                objectUniqueId=sim.drone_id,
                linkIndex=-1,
                forceObj=[0.5, 0.5, 0],  # Apply a lateral force
                posObj=[0, 0, 0],
                flags=p.LINK_FRAME,
            )
            print("INFO: Applied disturbance force")
        if ord("q") in keys and keys[ord("q")] & p.KEY_WAS_TRIGGERED:
            if sim.is_logging:
                sim.stop_logging()
            print("INFO: Quitting simulation")
            p.disconnect()
            break

        # Step simulation
        p.stepSimulation()
        loop_time = time.time() - loop_start
        if loop_time < timestep:
            time.sleep(timestep - loop_time)
