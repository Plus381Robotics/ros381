import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *
from ros381_tactics.ax12a import *
import time

tactic_state = 0

start_x = -1.057
start_y = 0.75
start_phi = -math.pi / 2
start_x_offset = 0.0

first_x = -1.057
first_y = 0.0
first_dir = 1

temp_x = 0.0
temp_y = 0.0
temp_dir = 0
temp_phi = 0.0


def load_t2():
    global start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset
    print("Tactic 2 loaded.")
    return start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset


def tactic_2():
    global tactic_state, temp_x, temp_y, temp_dir, temp_phi, first_x, first_y, first_dir

    match tactic_state:
        case 0:
            move_to_xy(first_x, first_y, first_dir, v_max=0.5)
            tactic_state = 10
        case 10:
            if move_success() or move_failed() or move_stacked():
                time.sleep(1)
                tactic_state = 12
            elif move_interrupted():
                time.sleep(0.2)
                tactic_state = 0
        case 12:
            move_to_xy(x=start_x, y=start_y, dir=-1, v_max=0.5)
            tactic_state = 15
        case 15:
            if move_success() or move_failed() or move_stacked():
                time.sleep(1)
                tactic_state = 0
            elif move_interrupted():
                time.sleep(0.2)
                tactic_state = 12
        case -1:
            print("Tactic 2 finished.")
    return tactic_state
