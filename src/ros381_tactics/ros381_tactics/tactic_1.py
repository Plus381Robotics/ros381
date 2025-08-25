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
            if move_success():
                tactic_state = 2
        case 2:
            move_to_xy(-0.4, -0.5, 1)
            tactic_state = 3
        case 3:
            if move_success():
                tactic_state = 4
            elif move_interrupted():
                tactic_state = 2
        case 4:
            rotate_to_phi(0.0)
            tactic_state = 5
        case 5:
            if move_success():
                tactic_state = 7
        case 7:
            move_to_xy(-1.25, 0.75, -1)
            tactic_state = 8
        case 8:
            if move_success():
                tactic_state = 9
            elif move_interrupted():
                tactic_state = 7
        case 9:
            move_to_xy(1.25, 0.0, -1)
            tactic_state = 10
        case 10:
            if move_success():
                tactic_state = -1
            elif move_interrupted():
                tactic_state = set_retry(1, 3, 9, 11)
        case 11:
            move_to_xy(0, 0, -1)
            tactic_state = 12
        case 12:
            if move_success():
                tactic_state = -1
        case -1:
            print("Tactic 1 finished.")
        case -10:
            tactic_state = exec_retry()
    return tactic_state
