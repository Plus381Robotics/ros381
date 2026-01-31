import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *

tactic_state = 0

start_x = -1.06
start_y = 0.735
start_phi = -math.pi / 2

first_x = -1.06
first_y = 0.2
first_dir = 1

temp_x = 0.0
temp_y = 0.0
temp_dir = 0
temp_phi = 0.0


def load_t3():
    global start_x, start_y, start_phi, first_x, first_y, first_dir
    print("Tactic 3 loaded.")
    return start_x, start_y, start_phi, first_x, first_y, first_dir


def tactic_3():
    global tactic_state, temp_x, temp_y, temp_dir, temp_phi, first_x, first_y, first_dir

    match tactic_state:
        case 0:
            move_to_xy(first_x, first_y, first_dir)
            tactic_state = 1
        case 1:
            if move_success():
                tactic_state = 2
        case 2:
            move_on_angle(dist=0.3, dir=1, phi=math.pi)
            tactic_state = 3
        case 3:
            if move_success() or move_stacked():
                tactic_state = 4
        case 4:
            move_to_xy(x=-1.0, y=0.2, dir=-1)
            tactic_state = 5
        case 5:
            if move_success():
                tactic_state = 6
        case 6:
            move_to_xy(x=-1.0, y=-0.6, dir=1)
            tactic_state = 7
        case 7:
            if move_success():
                tactic_state = 8
        case 8:
            move_on_angle(dist=0.3, dir=-1, phi=0)
            tactic_state = 9
        case 9:
            if move_success() or move_stacked():
                tactic_state = 10
        case -1:
            print("Tactic 3 finished.")
    return tactic_state
