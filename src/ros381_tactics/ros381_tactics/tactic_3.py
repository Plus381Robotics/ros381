import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *
from ros381_tactics.ax12a import *

tactic_state = 0

start_x = -1.06
start_y = 0.75
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
            tactic_state = 10
        case 10:
            if move_success():
                tactic_state = 11
        case 11:
            rotate_to_phi(phi=math.pi)
            tactic_state = 12
        case 12:
            if move_success():
                tactic_state = 15
        case 15:
            if snapshot_fsm(1, 47, 10) < 0:
                tactic_state = 20
        case 20:
            move_on_angle(dist=0.1, dir=1, phi=math.pi)
            tactic_state = 30
        case 30:
            if move_success() or move_stacked():
                tactic_state = 40
        case 40:
            state, position = lift(1, -1)
            if state < 0:
                if position > 900:
                    print("Stack NOT here!")
                else:
                    print("Stack here!")
                tactic_state = 50
        case 50:
            if lift_carry(1) < 0:
                tactic_state = 60
        case 60:
            move_on_angle(dist=0.2, dir=-1, phi=math.pi)
            tactic_state = 65
        case 65:
            if move_success():
                tactic_state = 70
        case 70:
            move_to_xy(x=-1.2, y=-0.1, dir=1)
            tactic_state = 75
        case 75:
            if move_success():
                tactic_state = 80
        case 80:
            if mechanism(1, 47) < 0:
                tactic_state = 90
        case 90:
            move_on_direction(dist=0.2, dir=-1)
            tactic_state = 95
        case 95:
            if move_success():
                tactic_state = 100
        case 100:
            move_to_xy(first_x, first_y, -1)
            tactic_state = 110
        case 110:
            if move_success():
                tactic_state = 120
        case 120:
            move_to_xy(start_x, start_y, -1)
            tactic_state = 130
        case 130:
            if move_success():
                tactic_state = -1
        case -1:
            print("Tactic 3 finished.")
    return tactic_state
