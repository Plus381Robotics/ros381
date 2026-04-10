import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *
from ros381_tactics.ax12a import *

tactic_state = 0

start_x = -1.067
start_y = 0.75
start_phi = -math.pi / 2
start_x_offset = 0.02

first_x = -1.06
first_y = 0.35
first_dir = 1

temp_x = 0.0
temp_y = 0.0
temp_dir = 0
temp_phi = 0.0


def load_t1():
    global start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset
    print("Tactic 1 loaded.")
    return start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset


def tactic_1():
    global tactic_state, temp_x, temp_y, temp_dir, temp_phi, first_x, first_y, first_dir

    match tactic_state:
        case 0:
            move_to_xy(first_x, first_y, first_dir)
            tactic_state = 10
        case 10:
            if move_success():
                tactic_state = 11
        case 11:
            move_to_xy(x=0.0, y=0.0, dir=1)
            tactic_state = 12
        case 12:
            if move_success():
                tactic_state = 13
        case 13:
            rotate_to_phi(phi= -math.pi*0.5)
            tactic_state = 14
        case 14:
            if move_success():
                tactic_state = -1
        case -1:
            print("Tactic 1 finished.")
    return tactic_state
