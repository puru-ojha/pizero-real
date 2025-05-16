#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
from openpi_client import image_tools

class DualRealSenseCamera:
    def __init__(self):
        # Initialize RealSense pipelines
        self.exterior_pipeline = rs.pipeline()
        self.wrist_pipeline = rs.pipeline()
        
        # Configure both cameras
        self.exterior_config = rs.config()
        self.wrist_config = rs.config()
        
        # # Get list of connected devices
        ctx = rs.context()
        devices = ctx.query_devices()
        # if len(devices) < 2:
        #     print(len(devices))
        #     raise RuntimeError(f"Found only {len(devices)} RealSense devices. Need 2 cameras.")
        
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

    def get_observation(self, state, task_instruction="Pick up the object"):
        """
        Capture images from both cameras and create observation dictionary
        Args:
            state: Robot state (joint angles, etc.)
            task_instruction: Text prompt for the policy
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
            
            # Process images according to required format
            exterior_processed = image_tools.convert_to_uint8(
                image_tools.resize_with_pad(exterior_image, 224, 224)
            )
            
            wrist_processed = image_tools.convert_to_uint8(
                image_tools.resize_with_pad(wrist_image, 224, 224)
            )
            
            # Create observation dictionary
            observation = {
                "observation/exterior_image_1_left": exterior_processed,
                "observation/wrist_image_left": wrist_processed,
                "observation/state": state,
                "prompt": task_instruction,
            }
            
            # Debug visualization
            self.debug_visualization(exterior_image, wrist_image, 
                                  exterior_processed, wrist_processed)
            
            return observation

        except Exception as e:
            print(f"Error capturing images: {str(e)}")
            return None

    def debug_visualization(self, original_exterior, original_wrist, 
                          processed_exterior, processed_wrist):
        """Show debug windows for all image processing steps"""
        # Show original images
        cv2.imshow('Original Exterior Camera', original_exterior)
        cv2.imshow('Original Wrist Camera', original_wrist)
        
        # Show processed images
        cv2.imshow('Processed Exterior Image (224x224)', processed_exterior)
        cv2.imshow('Processed Wrist Image (224x224)', processed_wrist)
        
        cv2.waitKey(1)

    def release(self):
        """Clean up resources"""
        self.exterior_pipeline.stop()
        self.wrist_pipeline.stop()
        cv2.destroyAllWindows()

def main():
    # Initialize cameras
    camera = DualRealSenseCamera()
    
    # Example state (replace with actual robot state)
    dummy_state = np.zeros(7)  # 7 joint angles
    
    try:
        while True:
            observation = camera.get_observation(
                state=dummy_state,
                task_instruction="Pick up the object"
            )
            
            if observation is None:
                print("Failed to get observation")
                continue
            
            # Break loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("Stopping camera feeds...")
    finally:
        camera.release()

if __name__ == "__main__":
    main()