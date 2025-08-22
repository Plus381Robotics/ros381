import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *

tactic_state = 0

start_x = -1.0
start_y = 0.0
start_phi = 0.0

first_x = -1.0
first_y = -0.5
first_dir = -1


def load_t1():
    global start_x, start_y, start_phi, first_x, first_y, first_dir
    print("Tactic 1 loaded.")
    return start_x, start_y, start_phi, first_x, first_y, first_dir


def tactic_1():
    global tactic_state

    match tactic_state:
        case 0:
            move_to_xy(first_x, first_y, first_dir)
            tactic_state = 1
        case 1:
            if get_move_result() == -1:
                tactic_state = 2
        case 2:
            move_to_xy(-0.4, -0.5, 1)
            tactic_state = 3
        case 3:
            if get_move_result() == -1:
                tactic_state = 4
            elif get_move_result() == -4:
                # get_GT().cancel_goal()
                tactic_state = 2
        case 4:
            rotate_to_phi(0.0)
            tactic_state = 5
        case 5:
            if get_move_result() == -1:
                tactic_state = 7
        case 7:
            move_to_xy(-1.25, 0.75, -1)
            tactic_state = 8
        case 8:
            if get_move_result() == -1:
                tactic_state = 9
            elif get_move_result() == -4:
                # get_GT().cancel_goal()
                tactic_state = 7
        case 9:
            move_to_xy(1.25, 0.0, -1)
            tactic_state = 10
        case 10:
            if get_move_result() == -1:
                tactic_state = -1
            elif get_move_result() == -4:
                # get_GT().cancel_goal()
                tactic_state = 9
        case -1:
            print("Tactic 1 finished.")
    return tactic_state
