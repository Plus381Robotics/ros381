import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *
from ros381_tactics.ax12a import *
import time


tactic_state = 0
prev_state = -1

start_x = -1.1
start_y = 0.723
start_phi = -math.pi / 2

first_x = 0.0
first_y = 0.5
first_dir = 1


def load_t7():
    global start_x, start_y, start_phi, first_x, first_y, first_dir
    print("Tactic 7 loaded.")
    return start_x, start_y, start_phi, first_x, first_y, first_dir


def tactic_7():
    global tactic_state, prev_state

    if prev_state != tactic_state:
        prev_state = tactic_state
        print("Tactic state = " + str(tactic_state))

    match tactic_state:
        case 0:
            if snapshot_fsm(1, 10) < 0:
                tactic_state = 10
        case 10:
            clear_cf()
            print("---------------------")
            time.sleep(5)
            tactic_state = 20
            
        case 20:
            if snapshot_fsm(-1, 10) < 0:
                tactic_state = 30
        case 30:
            clear_cb()
            print("---------------------")
            time.sleep(5)
            tactic_state = 0
        case -1:
            print("Tactic 7 finished.")
    return tactic_state
