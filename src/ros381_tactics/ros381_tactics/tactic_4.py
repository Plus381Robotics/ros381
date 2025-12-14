import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *
from ros381_tactics.ax12a import *

tactic_state = 0

start_x = -1.1
start_y = 0.723
start_phi = -math.pi/2

first_x = 0.0
first_y = 0.5
first_dir = 1


def load_t4():
    global start_x, start_y, start_phi, first_x, first_y, first_dir
    print("Tactic 4 loaded.")
    return start_x, start_y, start_phi, first_x, first_y, first_dir


def tactic_4():
    global tactic_state

    match tactic_state:
        case 0:
            ax_move(4, 511, 250, 50)
            tactic_state = 1
        case 10:
            if get_move_result() == -1:
                tactic_state = -1
        case -1:
            print("Tactic 4 finished.")
    return tactic_state
