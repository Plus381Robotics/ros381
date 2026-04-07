import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *
from ros381_tactics.ax12a import *

tactic_state = 0
prev_state = -1

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
#   mehanizam odradi
#   mehanizam resetuje
#   kursor dole
#   kursor gore


def tactic_6():
    global tactic_state, prev_state

    if not get_GT().consumed_front:
        print(f"Crate Stack position: ({get_GT().cs_front_x}, {get_GT().cs_front_y}, {get_GT().cs_front_phi})")
        for i in range(4):
            print(f"crate[{i}] color:", get_GT().crates_front[i])
        get_GT().consumed_front = True
    if prev_state != tactic_state:
        prev_state = tactic_state
        print("Tactic state = " + str(tactic_state))

    match tactic_state:
        case 0:
            if snapshot_fsm(1, 10) < 0:
                tactic_state = 10
        case 10:
            state, position = lift(1, -1)
            if state < 0:
                if position > 900:
                    print("Stack NOT here!")
                else:
                    print("Stack here!")
                tactic_state = 20
        case 20:
            if lift_carry(1) < 0:
                tactic_state = 30
        case 30:
            if mechanism(1) < 0:
                tactic_state = 31
        case 31:
            state, position = lift(-1, -1)
            if state < 0:
                if position > 900:
                    print("Stack NOT here!")
                else:
                    print("Stack here!")
                tactic_state = 32
        case 32:
            if lift_carry(-1) < 0:
                tactic_state = 33
        case 33:
            if mechanism(-1) < 0:
                tactic_state = 40
        case 40:
            if cursor(0) < 0:
                tactic_state = 50
        case 50:
            if cursor(1) < 0:
                tactic_state = 60
            tactic_state = 60
        case 60:
            if mechanism_reset(1) < 0:
                tactic_state = -1
        case -1:
            print("Tactic 6 finished.")
    return tactic_state
