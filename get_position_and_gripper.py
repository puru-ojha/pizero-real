# joint angles
#!/usr/bin/env python3
from xarm.wrapper import XArmAPI
import time

def move_and_check_positions():
    # Initialize the XArm API with the robot's IP address
    arm = XArmAPI('192.168.1.242')  # Replace with your xArm's IP address
    
    # Power on the robot
    arm.motion_enable(enable=True)
    arm.set_gripper_mode(0)
    arm.set_mode(0)  # Set to position control mode
    arm.set_gripper_enable(True)
    arm.set_gripper_speed(500)
    arm.set_state(state=0)  # Set to ready state
    
    # Wait for the robot to be ready
    time.sleep(0.5)
    
    # Define target joint angles (in degrees)
    target_angles = [0, 12, 0, 32, -22, 0, 0] 
    
    print("Moving to target angles...")
    # Move to target position
    ret = arm.set_servo_angle(angle=target_angles, speed=7, wait=True)

    # Wait for movement to complete
    time.sleep(1)
    
    # Get and print current joint angles
    ret = arm.get_servo_angle()
    if ret[0] == 0:  # Return code 0 means success
        angles = ret[1]
        print("\nTarget joint angles (degrees):")
        for i, angle in enumerate(target_angles):
            print(f"J{i+1}: {angle:.2f}°")
            
        print("\nActual joint angles (degrees):")
        for i, angle in enumerate(angles):
            print(f"J{i+1}: {angle:.2f}°")
    else:
        print("Error getting joint angles")

    # Get and print current end effector position
    ret = arm.get_position()
    if ret[0] == 0:
        x, y, z, roll, pitch, yaw = ret[1]
        print("\nEnd effector position:")
        print(f"Position (mm): X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
        print(f"Orientation (deg): Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={yaw:.2f}")
    else:
        print("Error getting end effector position")

    # Gripper control
    print("\nGripper Control:")
    
    # Get initial gripper position
    ret = arm.get_gripper_position()
    print(ret)
    if ret[0] == 0:
        print(f"Current gripper position: {ret[1]}mm")
    else:
        print("Error getting gripper position")

    # Open gripper (max position is typically around 850)
    print("\nOpening gripper...")
    arm.set_gripper_position(600, wait=True)
    time.sleep(1)
    
    # Get gripper position after opening
    ret = arm.get_gripper_position()
    if ret[0] == 0:
        print(f"Gripper position after opening: {ret[1]}mm")

    # Close gripper
    print("\nClosing gripper...")
    arm.set_gripper_position(0, wait=True)
    time.sleep(1)
    
    # Get gripper position after closing
    ret = arm.get_gripper_position()
    if ret[0] == 0:
        print(f"Gripper position after closing: {ret[1]}mm")
    
    # Clean up
    arm.disconnect()

if __name__ == "__main__":
    move_and_check_positions()