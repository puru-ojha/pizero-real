from xarm.wrapper import XArmAPI
import time
import numpy as np

IP = '192.168.1.242'  

arm = XArmAPI(IP)
arm.motion_enable(enable=True)
arm.set_mode(0) 
arm.set_state(state=0)  
arm.set_collision_sensitivity(5)

positions = []
data = []

with open("cart_pos.txt", "r") as file:

    for line in file:
        positions.append(eval(line.strip()))
        position = np.asarray((positions[0][:3]))*1000  # First 3 values are position (x, y, z)
        orientation = positions[0][3:]  # Next 3 values are orientation (roll, pitch, yaw)
        # print(positions)

        # print(positions)
        # print(position, orientation) 
        data.append((position, orientation))
        


def read_positions_and_orientations_from_file(filename):
    data = []
    with open(filename, 'r') as file:
        for line in file:
            values = list(map(float, line.strip().split())) 

            if len(values) != 6:
                print(f"Skipping invalid line: {line.strip()}")
                continue 

            position = values[:3]  # First 3 values are position (x, y, z)
            orientation = values[3:]  # Next 3 values are orientation (roll, pitch, yaw)

            data.append((position, orientation))

    return data


def move_arm_to_positions_and_orientations(arm, data):
    for position, orientation in data:
        print(f"Moving to position: {position}, orientation: {orientation}")

        arm.set_position(
            x=position[0], y=position[1], z=position[2],  # Position (x, y, z)
            roll=orientation[0], pitch=orientation[1], yaw=orientation[2],  # Orientation (roll, pitch, yaw)
            wait=True, 
            speed = 1, 
            motion_type = 1, 
            is_radian = True,
        )

        print(f"Current position:", arm.get_position())

        time.sleep(1)  

filename = 'cart_pos.txt'

# data = read_positions_and_orientations_from_file(filename)

# print(data)

move_arm_to_positions_and_orientations(arm, data)

arm.disconnect()