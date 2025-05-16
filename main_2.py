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
        print(position, orientation) 
        data.append((position, orientation))
        
        # break

# def read_positions_and_orientations_from_file(filename):
#     data = []
#     with open(filename, 'r') as file:
#         for line in file:
#             values = list(map(float, line.strip().split())) 

#             if len(values) != 6:
#                 print(f"Skipping invalid line: {line.strip()}")
#                 continue

#             position = values[:3]  # First 3 values are position (x, y, z)
#             orientation = values[3:]  # Next 3 values are orientation (roll, pitch, yaw)

#             data.append((position, orientation))

    # return data

def move_arm_slowly(arm, data, step_size=0.01, delay=0.1):

    _, current_pos_and_ori = arm.get_position()

    current_pos = current_pos_and_ori[:3]
    current_ori = current_pos_and_ori[3:]

    print("Current position and orientation:", current_ori , current_pos)

    for target_pos, target_ori in data:
        print(f"Moving to position: {target_pos}, orientation: {target_ori}")

        # pos_diff = np.array(target_pos) - np.array(current_pos)
        # ori_diff = np.array(target_ori) - np.array(current_ori)

        # num_steps = int(np.max(np.abs(np.concatenate((pos_diff, ori_diff)))) / step_size)

        # print(num_steps)
        # num_steps = 500
        
        # for step in range(num_steps):

        #     intermediate_pos = current_pos + (pos_diff * (step + 1) / num_steps)
        #     intermediate_ori = current_ori + (ori_diff * (step + 1) / num_steps)

        #     print(intermediate_pos, intermediate_ori)
    
        arm.set_position(
            x= target_pos[0], y= target_pos[1], z= target_pos[2],
            roll= target_ori[0], pitch= target_ori[1], yaw= target_ori[2],
            wait=True,
            speed=1,
            motion_type=1,
            is_radian=True,
        )

        time.sleep(delay)

        current_pos = target_pos
        current_ori = target_ori

filename = 'cart_pos.txt'


# print(data)
# data = read_positions_and_orientations_from_file(filename)

# move_arm_slowly(arm, data, step_size=0.01, delay=0.1)  # Adjust step_size and delay for speed

arm.disconnect()