import time
from ros381_tactics.movement import *
from ros381_tactics.ax12a import *
from ros381_tactics.tactic_0 import tactic_0, load_t0
from ros381_tactics.tactic_1 import tactic_1, load_t1
from ros381_tactics.tactic_2 import tactic_2, load_t2
from ros381_tactics.tactic_3 import tactic_3, load_t3
from ros381_tactics.tactic_4 import tactic_4, load_t4
from ros381_tactics.tactic_5 import tactic_5, load_t5
from ros381_tactics.tactic_6 import tactic_6, load_t6
from ros381_tactics.get_set import *


def hello_tactics():
    print("Individual tactics python module loaded.")
    return True


load_state = 0
chosen_tactic = -1

start_x = 0.0
start_y = 0.0
start_phi = 0.0
first_x = 0.0
first_y = 0.0
first_dir = 0


def execute_tactic():
    global chosen_tactic
    return globals()[f"tactic_{chosen_tactic}"]()


def load_tactic(GT, tactic_number, tactic_side):
    global load_state, chosen_tactic, start_x, start_y, start_phi, first_x, first_y, first_dir
    match load_state:
        case 0:
            set_GT(GT)
            load_ax_params()

            chosen_tactic = tactic_number
            set_side(tactic_side)
            start_x, start_y, start_phi, first_x, first_y, first_dir = globals()[
                f"load_t{tactic_number}"
            ]()
            start_x, start_phi = sided_coords(start_x, start_phi)
            if tactic_side == -1:
                print("Yellow side chosen.")
            else:
                print("Blue side chosen.")

            # TODO: vrati na 1
            load_state = -1
        case 1:
            GT.update_pose(start_x, start_y, start_phi, 111)
            GT.publish_pose_offset(start_x, start_y, start_phi)
            load_state = 2
        case 2:
            if get_update_pose_result() == -1:
                # TODO: parametar skip_ax_init
                load_state = 3
        case 3:
            GT.set_vacuum(False, False)
            load_state = 4
        case 4:
            if init_ax():
                load_state = 5
        case 5:
            rotate_to_xy(first_x, first_y, first_dir)
            load_state = 6
        case 6:
            if GT.move_result_ < 0:
                load_state = -1
    return load_state


def reset_tactic():
    global load_state
    if load_state != 0:
        get_GT().cancel_goal()
        load_state = 0
        print("Reseting load state")
