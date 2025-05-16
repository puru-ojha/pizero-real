#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
from openpi_client import image_tools

class SingleRealSenseCamera:
    def __init__(self):
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Get device
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) < 1:
            raise RuntimeError(f"No RealSense devices found.")
        
        # Enable streams for camera
        self.config.enable_device(devices[0].get_info(rs.camera_info.serial_number))
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start streaming
        print("Starting camera...")
        self.pipeline.start(self.config)
        print("Camera initialized successfully")

    def get_observation(self, state, task_instruction="Pick up the- object"):
        """
        Capture image from camera and create observation dictionary
        Args:
            state: Robot state (joint angles, etc.)
            task_instruction: Text prompt for the policy
        """
        try:
            # Get frames from camera
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                print("Error: Could not get color frame from camera")
                return None

            # Convert image to numpy array
            image = np.asanyarray(color_frame.get_data())
            
            # Process image according to required format
            processed_image = image_tools.convert_to_uint8(
                image_tools.resize_with_pad(image, 224, 224)
            )
            
            # Create observation dictionary
            observation = {
                "observation/image": processed_image,
                "observation/state": state,
                "prompt": task_instruction,
            }
            
            # Debug visualization
            self.debug_visualization(image, processed_image)
            
            return observation

        except Exception as e:
            print(f"Error capturing image: {str(e)}")
            return None

    def debug_visualization(self, original_image, processed_image):
        """Show debug windows for image processing steps"""
        cv2.imshow('Original Camera', original_image)
        cv2.imshow('Processed Image (224x224)', processed_image)
        cv2.waitKey(1)

    def release(self):
        """Clean up resources"""
        self.pipeline.stop()
        cv2.destroyAllWindows()

def main():
    # Initialize camera
    camera = SingleRealSenseCamera()
    
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
        print("Stopping camera feed...")
    finally:
        camera.release()

if __name__ == "__main__":
    main()