# State Machine draft for Demo 2

import smbus
import time
# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04



state_codes = {
    state_start : -10000,
    state_FOV_rotate : -10001,
    state_find_tape : -10002,
    state_turn_to_start : -10003,
    state_calc_dist_to_start : -10004,
    state_drive_to_start : -10005,
    state_calc_path_angle : -10006,
    state_turn_inline_to_path : -10007,
    state_calc_dist_to_end : -10008,
    state_drive_to_end : -10009,
    state_stop : -10010,
    }

# This is the start state. No code takes place here.
def state_start():
    return state_FOV_rotate

# This state primarily takes place on the arduino where the robot is rotated half its FOV clockwise. The flag
# rotateComplete will be set to -10000 and sent to the Pi when the robot has finished rotating
def state_FOV_rotate():
    # Send a state code to the Arduino to let it know which state to be in
    bus.write_byte_data(state_codes[state_FOV_rotate])


    return state_find_tape
