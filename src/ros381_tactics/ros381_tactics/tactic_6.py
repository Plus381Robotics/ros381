import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *
from ros381_tactics.ax12a import *

tactic_state = 0

start_x = -1.1
start_y = 0.723
start_phi = -math.pi / 2

first_x = 0.0
first_y = 0.5
first_dir = 1


def load_t6():
    global start_x, start_y, start_phi, first_x, first_y, first_dir
    print("Tactic 6 loaded.")
    return start_x, start_y, start_phi, first_x, first_y, first_dir


# TODO:
#   lift dole
#   lift nosi
#   mehanizam
#   kursor dole
#   kursor gore
#   lift gore
#   mehanizam
#   lift dole


def tactic_6():
    global tactic_state

    if not get_GT().consumed_back:
        print(f"Crate Stack position: ({get_GT().cs_back_x}, {get_GT().cs_back_y}, {get_GT().cs_back_phi})")
        for i in range(4):
            print(f"crate[{i}] color:", get_GT().crates_back[i])
        get_GT().consumed_back = True

    match tactic_state:
        case 0:
            if snapshot_fsm(-1, 47, 10) < 0:
                tactic_state = 10
        case 10:
            state, position = lift(-1, 0)
            if state < 0:
                if position < 900:
                    print("Stack NOT here!")
                else:
                    print("Stack here!")
                tactic_state = 20
        case 20:
            if lift_carry(-1) < 0:
                tactic_state = 30
        case 30:
            if mechanism(-1, 47):
                tactic_state = 40
        case 40:
            if cursor(0) < 0:
                tactic_state = 50
        case 50:
            if cursor(1) < 0:
                tactic_state = -1
        case -1:
            print("Tactic 6 finished.")
    return tactic_state
