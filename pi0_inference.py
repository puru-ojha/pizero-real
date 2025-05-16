#!/usr/bin/env python3
import time
import numpy as np
import cv2
import pyrealsense2 as rs
import os
import sys

from openpi_client import image_tools
from openpi_client.websocket_client_policy import WebsocketClientPolicy
from xarm.wrapper import XArmAPI

# =============================================================================
# Camera Input via Dual RealSense Cameras
# =============================================================================
class DualRealSenseCamera:
    def __init__(self):
        # Initialize RealSense pipelines
        self.exterior_pipeline = rs.pipeline()
        self.wrist_pipeline = rs.pipeline()
        
        # Configure both cameras
        self.exterior_config = rs.config()
        self.wrist_config = rs.config()
        
        # Get list of connected devices
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) < 2:
            raise RuntimeError(f"Found only {len(devices)} RealSense devices. Need 2 cameras.")
        
        # Enable streams for exterior camera (first device)
        self.exterior_config.enable_device(devices[0].get_info(rs.camera_info.serial_number))
        self.exterior_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Enable streams for wrist camera (second device)
        self.wrist_config.enable_device(devices[1].get_info(rs.camera_info.serial_number))
        self.wrist_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start streaming from both cameras
        print("Starting exterior camera...")
        self.exterior_pipeline.start(self.exterior_config)
        print("Starting wrist camera...")
        self.wrist_pipeline.start(self.wrist_config)
        print("Both cameras initialized successfully")

        safety_boundaries = {
            "x_min": 0.1, "x_max": 1.0,
            "y_min": -0.5, "y_max": 0.5,
            "z_min": 0.1, "z_max": 1.2
        }

    def get_observation(self, state, task_instruction="Pick up the object"):
        """
        Capture images from both cameras and create observation dictionary.
        
        Args:
            state (dict): a dictionary with keys "joint_position" and "gripper_position" 
                          containing the current robot state.
            task_instruction (str): Text prompt for the policy.
        """
        try:
            # Get frames from both cameras
            exterior_frames = self.exterior_pipeline.wait_for_frames()
            wrist_frames = self.wrist_pipeline.wait_for_frames()
            
            exterior_color = exterior_frames.get_color_frame()
            wrist_color = wrist_frames.get_color_frame()
            
            if not exterior_color or not wrist_color:
                print("Error: Could not get color frames from both cameras")
                return None

            # Convert images to numpy arrays
            exterior_image = np.asanyarray(exterior_color.get_data())
            wrist_image = np.asanyarray(wrist_color.get_data())
            # wrist_image = cv2.flip(wrist_image, -1)
            
            # Process images (resize with padding and convert to uint8)
            exterior_processed = image_tools.convert_to_uint8(
                image_tools.resize_with_pad(exterior_image, 224, 224)
            )
            wrist_processed = image_tools.convert_to_uint8(
                image_tools.resize_with_pad(wrist_image, 224, 224)
            )
            
            # Build an observation dictionary that includes images and robot state.
            observation = {
                "observation/exterior_image_1_left": exterior_processed,
                "observation/wrist_image_left": wrist_processed,
                "observation/joint_position": state["joint_position"],
                "observation/gripper_position": state["gripper_position"],
                "prompt": task_instruction,
            }
            
            # Optional: Debug visualization of original and processed images.
            self.debug_visualization(exterior_image, wrist_image, 
                                     exterior_processed, wrist_processed)
            
            return observation

        except Exception as e:
            print(f"Error capturing images: {str(e)}")
            return None

    def debug_visualization(self, original_exterior, original_wrist, processed_exterior, processed_wrist):
        """Show debug windows for original and processed images."""
        cv2.imshow('Original Exterior Camera', original_exterior)
        cv2.imshow('Original Wrist Camera', original_wrist)
        cv2.imshow('Processed Exterior Image (224x224)', processed_exterior)
        cv2.imshow('Processed Wrist Image (224x224)', processed_wrist)
        cv2.waitKey(1)

    def release(self):
        """Clean up RealSense pipelines and windows."""
        self.exterior_pipeline.stop()
        self.wrist_pipeline.stop()
        cv2.destroyAllWindows()

# =============================================================================
# Main Robot Control Class using XArm API and Camera Input
# =============================================================================
class CameraJointControl:
    def __init__(self, arm_ip='192.168.1.242', open_loop_horizon=10):
        print("Starting Camera Joint Control script...")
        self.open_loop_horizon = open_loop_horizon

        # Initialize the dual RealSense camera object
        self.camera = DualRealSenseCamera()

        # Initialize xArm API
        print("Connecting to xArm at", arm_ip)
        self.arm = XArmAPI(arm_ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_gripper_mode(0)
        self.arm.set_gripper_enable(True)
        self.arm.set_gripper_speed(500)
        self.arm.set_mode(0)   # Position control mode
        self.arm.set_state(state=0)  # Ready state
        time.sleep(0.5)
        
        # Joint names (for bookkeeping) and joint limits in degrees
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        self.joint_limits = {
            "joint1": (-180, 180),
            "joint2": (-90, 90),
            "joint3": (-90, 90),
            "joint4": (-180, 180),
            "joint5": (-180, 180),
            "joint6": (-180, 180),
            "joint7": (-180, 180)
        }
        self.velocity_limits = {
            "joint1": 180,
            "joint2": 180,
            "joint3": 180,
            "joint4": 180,
            "joint5": 180,
            "joint6": 180,
            "joint7": 180
        }
        
        # Violation/warning counters
        self.joint_limit_violation_count = 0
        self.self_collision_count = 0
        self.velocity_violation_count = 0
        self.singularity_count = 0
        
        # Record start time for performance reporting
        self.task_start_time = time.time()

        # Move arm to a default "home" position.
        self.move_to_default_position()

        # Connect to the policy server
        try:
            self.policy_client = WebsocketClientPolicy(host="10.4.25.44", port=8000)
            print("Policy server connected.")
        except Exception as e:
            print(f"Error connecting to policy server: {e}")
            self.policy_client = None

    # ----------------------- xArm API Wrappers -------------------------
    def get_servo_angles(self):
        """Return current joint angles (degrees) as a NumPy array."""
        ret = self.arm.get_servo_angle(is_radian =True)  # Expect (ret_code, [angles])
        # print(ret)
        if ret[0] == 0:
            return np.array(ret[1])
        else:
            print("Error getting servo angles, returning zeros.")
            return np.zeros(7)

    def set_servo_angles(self, angles):
        """Command the arm to move to the specified joint angles (in degrees)."""
        ret = self.arm.set_servo_angle(angle=angles, speed=0.5, wait=True,is_radian = True)
        # print(ret)
        # if ret[0] != 0:
        #     print("Error sending servo angles command.")

    def get_gripper_position(self):
        """Return current gripper position."""
        ret = self.arm.get_gripper_position()
        if ret[0] == 0:
            return [ret[1]]
        else:
            print("Error getting gripper position, returning 0.0.")
            return [0.0]
        

    def set_gripper_position(self, position):
        """Set gripper to the specified position."""
        ret = self.arm.set_gripper_position(position)
        # if ret[0] != 0:
        #     print("Error sending gripper command."

    def move_to_default_position(self):
        """Move the arm to a predefined home position."""
        print("Moving to default (home) position...")
        home_angles_deg = [0.3, -8.3, 8, 24.8, -24.7, -12.9, 0] 
        # Convert to radians
        home_angles = np.deg2rad(home_angles_deg).tolist()

        gripper_open = 0.04  # Adjust based on your gripper configuration
        self.set_servo_angles(home_angles)
        self.set_gripper_position(gripper_open)
        time.sleep(5)
        print("Arrived at default position.")

    # ------------------ Helper Functions (Transforms etc.) ------------------
    def get_transformation_matrix(self, a, d, alpha, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0, a],
            [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -d*np.sin(alpha)],
            [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha),  np.cos(alpha),  d*np.cos(alpha)],
            [0, 0, 0, 1]
        ], dtype=float)

    def compute_jacobian(self, joint_angles):
        # Compute DH parameters using joint angles (converted to radians)
        dh_params = np.array([
            [0.0,    0.333,    0.0,         np.deg2rad(joint_angles[0])],
            [0.0,    0.0,     -np.pi/2,     np.deg2rad(joint_angles[1])],
            [0.0,    0.316,    np.pi/2,     np.deg2rad(joint_angles[2])],
            [0.0825, 0.0,      np.pi/2,     np.deg2rad(joint_angles[3])],
            [-0.0825,0.384,   -np.pi/2,     np.deg2rad(joint_angles[4])],
            [0.0,    0.0,      np.pi/2,     np.deg2rad(joint_angles[5])],
            [0.088,  0.0,      np.pi/2,     np.deg2rad(joint_angles[6])],
            [0.0,    0.107,    0.0,         0.0]
        ], dtype=float)
        
        T_EE = np.eye(4)
        for i in range(dh_params.shape[0]):
            a, d, alpha, theta = dh_params[i]
            T_EE = np.dot(T_EE, self.get_transformation_matrix(a, d, alpha, theta))
        
        J = np.zeros((6, dh_params.shape[0]))
        T_current = np.eye(4)
        for i in range(dh_params.shape[0]):
            a, d, alpha, theta = dh_params[i]
            T_current = np.dot(T_current, self.get_transformation_matrix(a, d, alpha, theta))
            p = T_EE[:3, 3] - T_current[:3, 3]
            z = T_current[:3, 2]
            J[:3, i] = np.cross(z, p)
            J[3:, i] = z
        return J[:, :7]

    def check_joint_limits(self, target_angles):
        violation_count = 0
        modified_angles = []
        for joint_name, angle in zip(self.joint_names, target_angles):
            if joint_name in self.joint_limits:
                min_angle, max_angle = self.joint_limits[joint_name]
                if angle < min_angle:
                    violation_count += 1
                    print(f"Warning: {joint_name} angle {angle:.2f}° below min {min_angle:.2f}°, clipping.")
                    angle = min_angle
                elif angle > max_angle:
                    violation_count += 1
                    print(f"Warning: {joint_name} angle {angle:.2f}° above max {max_angle:.2f}°, clipping.")
                    angle = max_angle
            modified_angles.append(angle)
        self.joint_limit_violation_count += violation_count
        return np.array(modified_angles)

    def check_velocity_limit(self, action_velocities):
        violation_count = 0
        scale_factor = 1.0
        for joint_name, velocity in zip(self.joint_names, action_velocities[:7]):
            actual_velocity = abs(velocity * scale_factor)
            if joint_name in self.velocity_limits:
                v_limit = self.velocity_limits[joint_name]
                if actual_velocity > 0.8 * v_limit:
                    violation_count += 1
                    print(f"Warning: {joint_name} velocity {actual_velocity:.2f}°/s exceeds 80% of limit {0.8*v_limit:.2f}°/s")
        self.velocity_violation_count += violation_count
        return violation_count

    def check_self_collision(self, joint_angles):
        # Placeholder for collision checking; extend with an actual algorithm if needed.
        collision = False
        if collision:
            self.self_collision_count += 1
        return collision

    def check_singularity(self, joint_angles, singularity_threshold=0.01):
        J = self.compute_jacobian(joint_angles)
        _, singular_values, _ = np.linalg.svd(J)
        min_singular_value = singular_values[-1]
        if min_singular_value < singularity_threshold:
            print(f"Warning: Near singularity detected: smallest singular value = {min_singular_value:.6f}")
            self.singularity_count += 1
            return True
        return False

    def convert_action_to_angles_and_gripper(self, action_step):
        if isinstance(action_step, list):
            action_step = np.array(action_step)
        dt = 1 / 15.0  # Time step in seconds.
        current_angles = self.get_servo_angles()  # Current joint angles (degrees)
        scale_factor = 1.0  # Adjust scaling as needed
        # Integrate velocities to compute target angles.
        target_angles = current_angles + scale_factor * action_step[:7] * dt
        print(f"The difference between target and current is {target_angles - current_angles}")
        target_angles = self.check_joint_limits(target_angles)
        self.check_velocity_limit(action_step[:7])
        if self.check_self_collision(target_angles):
            print("Warning: Self collision detected for target angles!")
        if self.check_singularity(target_angles):
            print("Warning: Singularity condition reached in target angles!")
        # Process gripper command (last value)
        gripper_val = action_step[7]
        if gripper_val < 0.0:
            gripper_command = 0.0
        elif gripper_val > 1.0:
            gripper_command = 0.04
        else:
            gripper_command = gripper_val * 0.04
        return target_angles.tolist(), gripper_command
    
    def debug_observation_dict(self, observation):
        """Print the keys and shapes of values in the observation dictionary and save images."""
        print("\n=== Observation Dictionary Debug ===")
        
        # Create a timestamp for unique filenames
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        
        # Create directory if it doesn't exist
        save_dir = "observation_images"
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        
        # Debug print each component of the observation
        for key, value in observation.items():
            if isinstance(value, np.ndarray):
                print(f"\nKey: {key}")
                print(f"Shape: {value.shape}")
                print(f"Type: {value.dtype}")
                print(f"Value range: min={value.min()}, max={value.max()}, mean={value.mean():.2f}")
                
                # Save images if the array has image-like dimensions
                if len(value.shape) == 3 and value.shape[-1] in [1, 3]:
                    filename = f"{save_dir}/{timestamp}_{key.replace('/', '_')}.png"
                    cv2.imwrite(filename, value)
                    print(f"Saved image to: {filename}")
                    
            elif isinstance(value, (list, tuple)):
                print(f"\nKey: {key}")
                print(f"Type: {type(value)}")
                print(f"Length: {len(value)}")
                print(f"Values: {value}")
            else:
                print(f"\nKey: {key}")
                print(f"Type: {type(value)}")
                print(f"Value: {value}")
                
        print("\n================================")


    def control_loop(self):
        dt = 1 / 15.0
        action_chunk = None
        print("Starting control loop. Press Ctrl+C to exit.")
        while True:
            # Get current robot state
            current_angles = self.get_servo_angles()
            current_gripper = self.get_gripper_position()
            # print("angles", current_angles)
            # print("gripper", current_gripper)
            state = {"joint_position": current_angles, "gripper_position": current_gripper}
            
            # Obtain observation from the DualRealSenseCamera
            observation = self.camera.get_observation(state, task_instruction="Go near the blue box")
            if observation is None:
                print("Failed to get observation")
                time.sleep(dt)
                continue

            self.debug_observation_dict(observation)
            
            # Request a new action chunk if none is pending
            if self.policy_client is not None and action_chunk is None:
                start_inference = time.time()
                try:
                    inference_result = self.policy_client.infer(observation)
                    action_chunk = inference_result["actions"]
                    # Round each inner action vector for clarity.
                    action_chunk = [np.round(np.array(action), 3).tolist() for action in action_chunk]
                    end_inference = time.time()
                    print(f"Inference time: {end_inference - start_inference:.3f} seconds")
                except Exception as e:
                    print(f"Error during inference: {e}")
                    action_chunk = None
            
            # Execute the received actions in an open-loop fashion.
            if action_chunk is not None:
                print(f"Processing action chunk with {len(action_chunk)} actions (each with {len(action_chunk[0])} elements).")
                for idx in range(min(self.open_loop_horizon, len(action_chunk))):
                    action = action_chunk[idx]
                    print(f"Processing action index {idx}: {action}")
                    action = np.clip(action, -1, 1)
                    new_angles, gripper_command = self.convert_action_to_angles_and_gripper(action)
                    if idx == 0 or idx == 9:
                        print(f"Setting joint angles to: {new_angles} for action number {idx}")
                        print(f"Setting gripper position to: {gripper_command}")
                    
                    self.set_servo_angles(new_angles)
                    self.set_gripper_position(gripper_command)
                    time.sleep(dt)

                    if idx == 0 or idx == 9:
                        print(f"getting angles for action number {idx}")
                        print(self.get_servo_angles())
                        print(f"getting gripper")
                        print(self.get_gripper_position())
                
                total_time = time.time() - self.task_start_time
                print(f"Total execution time: {total_time:.3f} seconds")
                print(f"Joint limit violations: {self.joint_limit_violation_count}")
                print(f"Velocity limit violations: {self.velocity_violation_count}")
                print(f"Self collision count: {self.self_collision_count}")
                print(f"Singularity warnings: {self.singularity_count}")
                action_chunk = None  # Reset to request new inference.
            else:
                print("No valid action chunk available, waiting...")
                time.sleep(dt)

# =============================================================================
# Main Entry Point
# =============================================================================
if __name__ == '__main__':
    try:
        controller = CameraJointControl(arm_ip='192.168.1.242', open_loop_horizon=8)
        controller.control_loop()
    except KeyboardInterrupt:
        print("Exiting on user interrupt.")
    finally:
        controller.camera.release()
        controller.arm.disconnect()
