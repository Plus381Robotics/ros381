import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *

tactic_state = 0

start_x = -1.1
start_y = 0.723
start_phi = -math.pi/2

first_x = 0.0
first_y = 0.5
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
            if get_move_result() == -1:
                tactic_state = 2
        case 2:
            move_to_xy(0.5, 0.25, 1)
            tactic_state = 10
        case 10:
            if get_move_result() == -1:
                tactic_state = -1
        case -1:
            print("Tactic 2 finished.")
    return tactic_state
