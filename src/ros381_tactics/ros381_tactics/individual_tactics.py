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
from ros381_tactics.tactic_7 import tactic_7, load_t7
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

start_x_offset = 0.0


def execute_tactic():
    global chosen_tactic
    return globals()[f"tactic_{chosen_tactic}"]()


def load_tactic(GT, tactic_number, tactic_side):
    global load_state, chosen_tactic, start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset
    match load_state:
        case 0:
            set_GT(GT)
            load_ax_params()

            chosen_tactic = tactic_number
            set_side(tactic_side)
            start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset = globals()[
                f"load_t{tactic_number}"
            ]()
            start_x, start_phi = sided_coords(start_x, start_phi)
            tactic_side = -1
            print("Yellow side chosen.")
            load_state = 10
        # 1)
        case 10:
            move_to_xy(-0.9, 0.8, 1, v_max=0.2, w_max=3.14)
            load_state = 11
        case 11:
            if GT.move_result_ < 0:
                load_state = 12
        # TODO: ovde ide prvo resetovanje pozicije (x i phi):
        #    GT.update_pose(start_x, start_y, start_phi, 101)
        # if get_update_pose_result() == -1:
        # 2)
        case 12:
            move_to_xy(-1.3, 0.8, -1, v_max=0.2, w_max=3.14)
            load_state = 13
        case 13:
            if GT.move_result_ < 0:
                load_state = 14
        # 3)
        case 14:
            move_to_xy(-1.3, 1.0, 1, v_max=0.2, w_max=3.14)
            load_state = 15
        case 15:
            if GT.move_result_ < 0:
                load_state = 16
        # TODO: ovde ide drugo resetovanje pozicije (y i phi):
        #    GT.update_pose(start_x, start_y, start_phi + math.pi*0.5, 011)
        # if get_update_pose_result() == -1:
        # 4)
        case 16:
            move_to_xy(-1.3, 0.8, -1, v_max=0.2, w_max=3.14)
            load_state = 17
        case 17:
            if GT.move_result_ < 0:
                load_state = 18
        # 5)
        case 18:
            rotate_to_phi(0.0)
            load_state = 19
        case 19:
            if GT.move_result_ < 0:
                load_state = -1
                
    return load_state


def reset_tactic():
    global load_state
    if load_state != 0:
        get_GT().cancel_goal()
        load_state = 0
        print("Reseting load state")
