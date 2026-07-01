import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *

tactic_state = 0
prev_state = -1

# robot je sirok 0.2726 m
# ovo su zapravo stack pozicije
start_x = -1.0363
start_y = 0.8637
start_phi = 0.0
start_x_offset = 0.0

first_x = 0.0
first_y = 0.0
first_dir = 1

temp_x = 0.0
temp_y = 0.0
temp_dir = 0
temp_phi = 0.0

v_des = 1.0
# I     Sporo:
#           v = 0.2
# II    Srednje:
#           v = 0.5
# III   Brzo:
#           v = 1.0


def load_t2():
    global start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset
    print("Tactic 2 loaded.")
    return start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset


def tactic_2():
    global tactic_state, temp_x, temp_y, temp_dir, temp_phi, first_x, first_y, first_dir, prev_state, v_des

    if prev_state != tactic_state:
        prev_state = tactic_state
        print("---------------------------------")
        print("Tactic state = " + str(tactic_state))
        print("Current time = " + str(get_GT().time))

    match tactic_state:
        # 1)
        case 0:
            rotate_to_phi(-math.pi * 0.5, w_max=1.57)
            tactic_state = 1
        case 1:
            if move_success():
                tactic_state = 10
        case 10:
            move_on_curve(x=-1.3, y=-0.8, phi=-math.pi, dir=1, v_max=v_des)
            tactic_state = 11
        case 11:
            if move_success():
                tactic_state = 12
        case 12:
            move_to_xy(x=-1.5, y=-0.8, dir=1, v_max=0.2, w_max=1.57)
            tactic_state = 13
        case 13:
            if move_stacked():
                # TODO: Print timestamp and current position
                tactic_state = 14
        case 14:
            move_to_xy(x=-1.3, y=-0.8, dir=-1, v_max=0.2, w_max=1.57)
            tactic_state = 15
        case 15:
            if move_success():
                tactic_state = 20
        # 2)
        case 20:
            move_to_xy(x=-1.3, y=-1.0, dir=1, v_max=0.2, w_max=1.57)
            tactic_state = 21
        case 21:
            if move_stacked():
                # TODO: Print timestamp and current position
                tactic_state = 22
        case 22:
            move_to_xy(x=-1.3, y=-0.8, dir=-1, v_max=0.2, w_max=1.57)
            tactic_state = 23
        case 23:
            if move_success():
                tactic_state = 24
        case 24:
            rotate_to_phi(math.pi * 0.5, w_max=1.57)
            tactic_state = 25
        case 25:
            if move_success():
                tactic_state = 30
        # 3)
        case 30:
            move_on_curve(x=1.3, y=-0.8, phi=-math.pi * 0.5, dir=1, v_max=v_des)
            tactic_state = 31
        case 31:
            if move_success():
                tactic_state = 32
        case 32:
            move_to_xy(x=1.3, y=-1.0, dir=1, v_max=0.2, w_max=1.57)
            tactic_state = 33
        case 33:
            if move_stacked():
                # TODO: Print timestamp and current position
                tactic_state = 34
        case 34:
            move_to_xy(x=1.3, y=-0.8, dir=-1, v_max=0.2, w_max=1.57)
            tactic_state = 35
        case 35:
            if move_success():
                tactic_state = 40
        # 4)
        case 40:
            move_to_xy(x=1.5, y=-0.8, dir=1, v_max=0.2, w_max=1.57)
            tactic_state = 41
        case 41:
            if move_stacked():
                # TODO: Print timestamp and current position
                tactic_state = 42
        case 42:
            move_to_xy(x=-1.3, y=-0.8, dir=-1, v_max=0.2, w_max=1.57)
            tactic_state = 43
        case 43:
            if move_success():
                tactic_state = 44
        case 44:
            rotate_to_phi(math.pi, w_max=1.57)
            tactic_state = 45
        case 45:
            if move_success():
                tactic_state = 50
        # 5)
        case 50:
            move_on_curve(x=1.3, y=0.8, dir=1, v_max=v_des)
            tactic_state = 51
        case 51:
            if move_success():
                tactic_state = 52
        case 52:
            move_to_xy(x=1.3, y=1.0, dir=1, v_max=0.2, w_max=1.57)
            tactic_state = 53
        case 53:
            if move_stacked():
                # TODO: Print timestamp and current position
                tactic_state = 54
        case 54:
            move_to_xy(x=1.3, y=0.8, dir=-1, v_max=0.2, w_max=1.57)
            tactic_state = 55
        case 55:
            if move_success():
                tactic_state = 60
        # 6)
        case 60:
            move_to_xy(x=1.5, y=0.8, dir=1, v_max=0.2, w_max=1.57)
            tactic_state = 61
        case 61:
            if move_stacked():
                # TODO: Print timestamp and current position
                tactic_state = 62
        case 62:
            move_to_xy(x=1.3, y=0.8, dir=-1, v_max=0.2, w_max=1.57)
            tactic_state = 63
        case 63:
            if move_success():
                tactic_state = 64
        case 64:
            rotate_to_phi(-math.pi * 0.5, w_max=1.57)
            tactic_state = 65
        case 65:
            if move_success():
                tactic_state = 70
        # 7)
        case 70:
            move_on_curve(x=-1.3, y=0.8, phi=math.pi * 0.5, dir=1, v_max=v_des)
            tactic_state = 71
        case 71:
            if move_success():
                tactic_state = 72
        case 72:
            move_to_xy(x=-1.3, y=1.0, dir=1, v_max=0.2, w_max=1.57)
            tactic_state = 73
        case 73:
            if move_stacked():
                # TODO: Print timestamp and current position
                tactic_state = 74
        case 74:
            move_to_xy(x=-1.3, y=0.8, dir=-1, v_max=0.2, w_max=1.57)
            tactic_state = 75
        case 75:
            if move_success():
                tactic_state = 80
        # 8)
        case 80:
            move_to_xy(x=-1.5, y=0.8, dir=1, v_max=0.2, w_max=1.57)
            tactic_state = 81
        case 81:
            if move_stacked():
                # TODO: Print timestamp and current position
                tactic_state = -1

        case -1:
            print("Tactic 2 finished.")
    return tactic_state
