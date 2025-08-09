from ros381_tactics.movement import *
from ros381_tactics.tactic_0 import tactic_0, load_t0
from ros381_tactics.tactic_1 import tactic_1, load_t1
from ros381_tactics import gt_holder


def hello_tactics():
    print("Individual tactics python module loaded.")
    return True


load_init = False
chosen_tactic = -1
chosen_side = 0
first_x = 0
first_y = 0
first_dir = 0


def execute_tactic():
    global chosen_tactic, chosen_side
    return globals()[f"tactic_{chosen_tactic}"](chosen_side)


def load_tactic(GT, tactic_number, tactic_side):
    global load_init, chosen_tactic, chosen_side
    if not load_init:
        gt_holder.set_GT(GT)
        load_init = True
        chosen_tactic = tactic_number
        chosen_side = tactic_side
        first_x, first_y, first_dir = globals()[f"load_t{tactic_number}"](tactic_side)
        rotate_to_xy(first_x, first_y, first_dir)
    return GT.move_result_
