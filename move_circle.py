#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Move Circle
"""

import os
import sys
import time
import numpy as np

# sys.path.append(os.path.join(os.path.dirname(_file_), '../../..'))

from xarm.wrapper import XArmAPI


#######################################################
"""
Just for test example
"""
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = '192.168.1.242'  
        # ip = input('Please input the xArm ip address:')
        # if not ip:
        #     print('input error, exit')
        #     sys.exit(1)
########################################################


arm = XArmAPI(ip, is_radian=True)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.reset(wait=True)

print("current position and orientation", arm.get_position()[1])
positions = []

data = []

with open("cart_pos.txt", "r") as file:
    for line in file:
        coords = eval(line.strip())   # Convert the line to a Python list/tuple
        position = np.asarray(coords[:3]) * 1000  # First 3 values -> position (in mm)
        orientation = coords[3:]                 # Next 3 values -> orientation (roll, pitch, yaw)
        print(position, orientation)
        data.append((position, orientation))



for target_pos, target_ori in data:
    print(f"Moving to position: {target_pos}, orientation: {target_ori}")
    poses = [target_pos[0], target_pos[1], target_pos[2], target_ori[0], target_ori[1], target_ori[2]]

    ret = arm.set_position(*poses, speed=10, mvacc=100, wait=True)
    # print('set_position, ret: {}'.format(ret))

    print("current position and orientation", arm.get_position()[1])

    # time.sleep(2)

print(data)

arm.reset(wait=True)
arm.disconnect()