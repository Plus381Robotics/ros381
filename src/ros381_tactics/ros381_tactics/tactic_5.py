import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *

tactic_state = 0
prev_state = -1

robot_length = 0.2726
robot_width = 0.314
# koordinatni sistem krece iz donjeg coska
start_x = 0.0
start_y = 0.0
edge_offset = 0.05
away_from_edge = 0.25

# I     Sporo:
#           v = 0.5, w = 1.57
# v_des = 0.5
# w_des = 1.57
# III   Brzo:
#           v = 1.0, w = 3.14
v_des = 1.0
w_des = 3.14


def load_t5():
    global robot_length, robot_width, v_des, w_des, start_x, start_y, edge_offset
    start_x = 0.6 - robot_width / 2
    start_y = 2.0 - robot_length / 2
    start_phi = -math.pi / 2
    print("Tactic 5 loaded - Straight sequence V2")
    print(f"Starting position = ( {start_x}, {start_y}, {start_phi} )")
    print(f"Desired velocities = ( {v_des}, {w_des} )")
    print(f"Edge offset  = {edge_offset}")
    return start_x, start_y, start_phi


def tactic_5():
    global tactic_state, prev_state, v_des, w_des, robot_length, robot_width, start_x, start_y, away_from_edge

    if prev_state != tactic_state:
        prev_state = tactic_state
        print("---------------------------------")
        print("Tactic state = " + str(tactic_state))
        print("Current time = " + str(get_GT().time))

    match tactic_state:
        case 0:
            move_to_xy(
                x=start_x,
                y=0.0 + robot_length / 2 - edge_offset,
                dir=1,
                v_max=v_des,
                w_max=w_des,
            )
            tactic_state = 11
        case 11:
            if move_stacked():
                tactic_state = 20
            elif move_success():
                tactic_state = 20
                print("DID NOT STACK!")

        case 20:
            move_to_xy(
                x=start_x,
                y=away_from_edge,
                dir=-1,
                v_max=v_des,
                w_max=w_des,
            )
            tactic_state = 21
        case 21:
            if move_success():
                tactic_state = 30

        case 30:
            move_to_xy(
                x=0.0 + robot_length / 2 - edge_offset,
                y=away_from_edge,
                dir=-1,
                v_max=v_des,
                w_max=w_des,
            )
            tactic_state = 31
        case 31:
            if move_stacked():
                tactic_state = 40
            elif move_success():
                tactic_state = 40
                print("DID NOT STACK!")

        case 40:
            move_to_xy(
                x=3.0 - robot_length / 2 + edge_offset,
                y=away_from_edge,
                dir=1,
                v_max=v_des,
                w_max=w_des,
            )
            tactic_state = 41
        case 41:
            if move_stacked():
                tactic_state = 50
            elif move_success():
                tactic_state = 50
                print("DID NOT STACK!")

        case 50:
            move_to_xy(
                x=3.0 - away_from_edge,
                y=away_from_edge,
                dir=-1,
                v_max=v_des,
                w_max=w_des,
            )
            tactic_state = 51
        case 51:
            if move_success():
                tactic_state = 60

        case 60:
            move_to_xy(
                x=3.0 - away_from_edge,
                y=0.0 + robot_length / 2 - edge_offset,
                dir=-1,
                v_max=v_des,
                w_max=w_des,
            )
            tactic_state = 61
        case 61:
            if move_stacked():
                tactic_state = 70
            elif move_success():
                tactic_state = 70
                print("DID NOT STACK!")

        case 70:
            move_to_xy(
                x=3.0 - away_from_edge,
                y=2.0 - robot_length / 2 + edge_offset,
                dir=1,
                v_max=v_des,
                w_max=w_des,
            )
            tactic_state = 71
        case 71:
            if move_stacked():
                tactic_state = 80
            elif move_success():
                tactic_state = 80
                print("DID NOT STACK!")

        case 80:
            move_to_xy(
                x=3.0 - away_from_edge,
                y=1.3,
                dir=-1,
                v_max=v_des,
                w_max=w_des,
            )
            tactic_state = 81
        case 81:
            if move_success():
                tactic_state = 90

        case 90:
            move_to_xy(
                x=3.0 - robot_length / 2 + edge_offset,
                y=1.3,
                dir=-1,
                v_max=v_des,
                w_max=w_des,
            )
            tactic_state = 91
        case 91:
            if move_stacked():
                tactic_state = 100
            elif move_success():
                tactic_state = 100
                print("DID NOT STACK!")

        case 100:
            move_to_xy(
                x=0.0 + robot_length / 2 - edge_offset,
                y=1.3,
                dir=1,
                v_max=v_des,
                w_max=w_des,
            )
            tactic_state = 101
        case 101:
            if move_stacked():
                tactic_state = 110
            elif move_success():
                tactic_state = 110
                print("DID NOT STACK!")

        case 110:
            move_to_xy(
                x=0.0 + away_from_edge,
                y=1.3,
                dir=-1,
                v_max=v_des,
                w_max=w_des,
            )
            tactic_state = 111
        case 111:
            if move_success():
                tactic_state = 120

        case 120:
            move_to_xy(
                x=0.0 + away_from_edge,
                y=2.0 - robot_length / 2 + edge_offset,
                dir=1,
                v_max=v_des,
                w_max=w_des,
            )
            tactic_state = 121
        case 121:
            if move_stacked():
                tactic_state = -1
            elif move_success():
                tactic_state = -1
                print("DID NOT STACK!")

        case -1:
            print("Tactic 5 finished.")
    return tactic_state
