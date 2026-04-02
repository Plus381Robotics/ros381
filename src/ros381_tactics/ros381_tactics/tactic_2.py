import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *

tactic_state = 0

start_x = 0.0
start_y = 0.0
start_phi = 0.0

first_x = 0.0
first_y = -0.5
first_dir = 1


def load_t2():
    global start_x, start_y, start_phi, first_x, first_y, first_dir
    print("Tactic 2 loaded.")
    return start_x, start_y, start_phi, first_x, first_y, first_dir


def tactic_2():
    global tactic_state

    match tactic_state:
        case 0:
            move_to_xy(first_x, first_y, first_dir)
            tactic_state = 1
        case 1:
            if move_success():
                tactic_state = 2
        case 2:
            move_to_xy(0.0, 0.0, -1)
            tactic_state = 3
        case 3:
            if move_success():
                tactic_state = 4
        case 4:
            move_on_direction(dist= 0.4, dir= 1)
            tactic_state = 5
        case 5:
            if move_success():
                tactic_state = 6
        case 6:
            move_on_angle(dist= 0.2, dir= 1, phi= 0.0)
            tactic_state = 7
        case 7:
            if move_success():
                tactic_state = -1
        case -1:
            print("Tactic 2 finished.")
    return tactic_state
