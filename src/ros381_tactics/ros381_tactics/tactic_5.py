import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *

tactic_state = 0
prev_state = -1

# robot je sirok 0.2726 m
# ovo su zapravo stack pozicije
start_x = -1.0363
start_y = 0.8637
start_phi = 0.0
start_x_offset = 0.0

first_x = 0.0
first_y = 0.0
first_dir = 1

temp_x = 0.0
temp_y = 0.0
temp_dir = 0
temp_phi = 0.0

v_des = 0.2
w_des = 1.57


def load_t5():
    global start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset
    print("Tactic 5 loaded - Straight test sequence")
    return start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset


def tactic_5():
    global tactic_state, temp_x, temp_y, temp_dir, temp_phi, first_x, first_y, first_dir, prev_state, v_des, w_des

    if prev_state != tactic_state:
        prev_state = tactic_state
        print("---------------------------------")
        print("Tactic state = " + str(tactic_state))
        print("Current time = " + str(get_GT().time))

    match tactic_state:
        case 0:
            move_to_xy(x=-1.3, y=-0.5, dir=1, v_max=v_des, w_max=w_des)
            tactic_state = 11
        case 11:
            if move_success():
                tactic_state = 30
        case 30:
            move_to_xy(x=-1.0, y=-0.5, dir=1, v_max=v_des, w_max=w_des)
            tactic_state = 31
        case 31:
            if move_success():
                tactic_state = 50
        case 50:
            move_to_xy(x=-1.0, y=0.5, dir=1, v_max=v_des, w_max=w_des)
            tactic_state = 51
        case 51:
            if move_success():
                tactic_state = 70
        case 70:
            move_to_xy(x=-1.3, y=0.5, dir=1, v_max=v_des, w_max=w_des)
            tactic_state = 71
        case 71:
            if move_success():
                tactic_state = -1

        case -1:
            print("Tactic 5 finished.")
    return tactic_state
