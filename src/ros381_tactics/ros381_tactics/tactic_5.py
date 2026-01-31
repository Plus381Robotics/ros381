import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *

tactic_state = 0

start_x = -1.06
start_y = 0.735
start_phi = -math.pi / 2

first_x = -1.06
first_y = 0.4
first_dir = 1

temp_x = 0.0
temp_y = 0.0
temp_dir = 0
temp_phi = 0.0


def load_t5():
    global start_x, start_y, start_phi, first_x, first_y, first_dir
    print("Tactic 5 loaded.")
    return start_x, start_y, start_phi, first_x, first_y, first_dir


def tactic_5():
    global tactic_state, temp_x, temp_y, temp_dir, temp_phi, first_x, first_y, first_dir

    match tactic_state:
        case 0:
            move_on_direction(dist=0.4, dir=1)
            tactic_state = 2
        case 2:
            if move_success():
                tactic_state = 10
        case 10:
            move_to_xy(x=0.35, y=0.15, dir=1)
            tactic_state = 12
        case 12:
            if move_success():
                tactic_state = 20
            elif move_interrupted():
                tactic_state = 20
        case 20:
            move_to_xy(x=0.35, y=-0.0625, dir=1)
            tactic_state = 22
        case 22:
            if move_success():
                tactic_state = 30
        case 30:
            move_to_xy(x=0.4, y=-0.45, dir=1)
            tactic_state = 32
        case 32:
            if move_success():
                tactic_state = 40
        case 40:
            move_on_angle(dist=0.35, dir=-1, phi=math.pi / 2)
            tactic_state = 42
        case 42:
            if move_success() or move_stacked():
                tactic_state = 50
        case 50:
            move_on_direction(dist=0.15, dir=1)
            tactic_state = 52
        case 52:
            if move_success():
                tactic_state = 55
        case 55:
            move_to_xy(x=-0.5, y=-0.5, dir=1)
            tactic_state = 57
        case 57:
            if move_success():
                tactic_state = 60
        case 60:
            move_to_xy(x=-1.2, y=-0.2, dir=1)
            tactic_state = 62
        case 62:
            if move_success():
                tactic_state = 70
        case 70:
            move_on_angle(dist=0.2, dir=1, phi=math.pi)
            tactic_state = 72
        case 72:
            if move_success() or move_stacked():
                tactic_state = 75
        case 75:
            move_on_direction(dist=0.15, dir=-1)
            tactic_state = 77
        case 77:
            if move_success():
                tactic_state = 80
        case 80:
            move_to_xy(x=-0.735, y=-0.25, dir=-1)
            tactic_state = 82
        case 82:
            if move_success():
                tactic_state = 90
        case 90:
            temp_x = -1.0
            temp_y = -0.6
            temp_dir = 1
            rotate_to_xy(x=temp_x, y=temp_y, dir=temp_dir)
            tactic_state = 92
        case 92:
            if move_success():
                tactic_state = 100
        case 100:
            move_to_xy(x=temp_x, y=temp_y, dir=temp_dir)
            tactic_state = 102
        case 102:
            if move_success():
                tactic_state = 110
        case 110:
            move_on_angle(dist=0.3, dir=1, phi=math.pi)
            tactic_state = 112
        case 112:
            if move_success() or move_stacked():
                tactic_state = 115
        case 115:
            move_on_direction(dist=0.15, dir=-1)
            tactic_state = 117
        case 117:
            if move_success():
                tactic_state = 120
        case 120:
            move_to_xy(x=-0.4, y=-0.5, dir=-1)
            tactic_state = 122
        case 122:
            if move_success():
                tactic_state = 130
        case 130:
            move_on_angle(dist=0.3, dir=-1, phi=math.pi / 2)
            tactic_state = 132
        case 132:
            if move_success() or move_stacked():
                tactic_state = 140
        case 140:
            move_on_direction(dist=0.2, dir=1)
            tactic_state = 142
        case 142:
            if move_success():
                tactic_state = 150
        case 150:
            tactic_state = -1
        case -1:
            print("Tactic 5 finished.")
    return tactic_state
