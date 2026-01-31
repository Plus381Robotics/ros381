import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *

tactic_state = 0

start_x = -1.06
start_y = 0.735
start_phi = -math.pi/2

first_x = -1.06
first_y = 0.4
first_dir = 1


def load_t5():
    global start_x, start_y, start_phi, first_x, first_y, first_dir
    print("Tactic 5 loaded.")
    return start_x, start_y, start_phi, first_x, first_y, first_dir


def tactic_5():
    global tactic_state

    match tactic_state:
        case 0:
            # move_to_xy(first_x, first_y, first_dir)
            move_on_direction(dist = 0.4, dir = 1)
            tactic_state = 10
        case 10:
            if move_success():
                tactic_state = 20
        case 20:
            move_to_xy(x = 0.35, y = 0.15, dir =1 )
            tactic_state = 30
        case 30:
            if move_success():
                tactic_state = 40
            elif move_interrupted():
                tactic_state = 40
        case 40:
            move_to_xy(x = 0.35, y = -0.0625, dir = 1)
            tactic_state = 50
        case 50:
            if move_success():
                tactic_state = 60
        case 60:
            move_to_xy(x = 0.4, y = -0.5, dir = 1)
            tactic_state = 70
        case 70:
            if move_success():
                tactic_state = 80
        case 80:
            move_on_angle(dist = 0.4, dir = -1, phi = math.pi/2)
            tactic_state = 90
        case 90:
            if move_success() or move_stacked():
                tactic_state = 100
        case -1:
            print("Tactic 5 finished.")
    return tactic_state
