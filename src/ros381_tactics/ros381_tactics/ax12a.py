from ros381_tactics.get_set import *

# pylint: disable=import-error
import ros381_tactics_py  # type: ignore

# pylint: enable=import-error
import math

init_state = 0


def init_ax():
    global init_state
    match init_state:
        case 0:
            # Lift gore
            ax_hybrid_move(15, 1000, 0.2, -200)
            init_state = 1
        case 1:
            if get_ax_hybrid_move_result() < 0:
                print(f"ID 15 reached position: {get_ax_hybrid_end_position()}")
                init_state = 2
        case 2:
            # Lift dole
            ax_hybrid_move(15, 1000, 0.2, 200)
            init_state = 3
        case 3:
            if get_ax_hybrid_move_result() < 0:
                print(f"ID 15 reached position: {get_ax_hybrid_end_position()}")
                init_state = 4
        case 4:
            # Clanak 1
            ax_move(11, 0, 300, 50)
            init_state = 5
        case 5:
            if get_ax_move_result() < 0:
                init_state = 6
        case 6:
            # Clanak 2
            ax_move(12, 0, 300, 50)
            init_state = 7
        case 7:
            if get_ax_move_result() < 0:
                init_state = 8
        case 8:
            # Clanak 3
            ax_move(13, 0, 300, 50)
            init_state = 9
        case 9:
            if get_ax_move_result() < 0:
                init_state = 10
        case 10:
            # Clanak 4
            ax_move(14, 0, 300, 50)
            init_state = 11
        case 11:
            if get_ax_move_result() < 0:
                init_state = 12
        case 12:
            ax_bulk_move(
                [
                    (11, 511, 1000, 100),
                    (12, 511, 1000, 100),
                    (13, 511, 1000, 100),
                    (14, 511, 1000, 100),
                    (15, 511, 1000, 100),
                ]
            )
            init_state = 99
        case 99:
            if get_ax_bulk_move_result() < 0:
                init_state = -1
        case -1:
            return True
    return False

cursor_state = 0
cursor_id = 6


def cursor(position):
    global cursor_state, cursor_id
    match cursor_state:
        case 0:
            if position == 1:
                cursor_state = 10
            else:
                cursor_state = 20
        case 10:
            ax_move(cursor_id, 950, 1000, 50)
            cursor_state = 11
        case 11:
            if get_ax_move_result() < 0:
                cursor_state = -1
        case 20:
            ax_hybrid_move(cursor_id, 1000, 0.2, -200)
            cursor_state = 21
        case 21:
            if get_ax_hybrid_move_result() < 0:
                cursor_state = 22
        case 22:
            cursor_pos = get_ax_hybrid_end_position()
            print ("Cursor reached position " + str(cursor_pos))
            cursor_state = -1
        case -1:
            cursor_state = 0
    return cursor_state


lift_state = 0
lift_id = 5
lift_pos = 511
lift_dpos = 200
lift_retpos = -1


def lift(side, state):
    global lift_state, lift_id, lift_pos, lift_dpos, lift_retpos
    match lift_state:
        case 0:
            lift_retpos = -1
            if side == 1:
                lift_id = 15
            else:
                lift_id = 5
            if state == 1:
                lift_pos = 900
                lift_dpos = 200
            else:
                lift_pos = 300
                lift_dpos = -200
            lift_state = 1
        case 1:
            ax_move(lift_id, lift_pos, 1000, 20)
            lift_state = 2
        case 2:
            if get_ax_move_result() < 0:
                lift_state = 3
        case 3:
            ax_hybrid_move(lift_id, 1000, 0.2, lift_dpos)
            lift_state = 4
        case 4:
            if get_ax_hybrid_move_result() < 0:
                lift_state = 5
        case 5:
            lift_retpos = get_ax_hybrid_end_position()
            lift_state = -1
        case -1:
            lift_state = 0
    return lift_state, lift_retpos


def lift_carry(side):
    global lift_id, lift_state
    match lift_state:
        case 0:
            if side == 1:
                lift_id = 15
            else:
                lift_id = 5
            lift_state = 1
        case 1:
            ax_move(lift_id, 400, 1000, 20)
            lift_state = 2
        case 2:
            if get_ax_move_result() < 0:
                lift_state = -1
        case -1:
            lift_state = 0
    return lift_state


def get_ax_move_result():
    return get_GT().ax_move_result_


def get_ax_bulk_move_result():
    return get_GT().ax_bulk_move_result_


def get_ax_hybrid_move_result():
    return get_GT().ax_hybrid_move_result_


def get_ax_hybrid_end_position():
    return get_GT().ax_hybrid_end_position_


def ax_move(id, position, velocity, position_tolerance):
    GT = get_GT()
    goal = ros381_tactics_py.AxMoveGoal()
    goal.id = id
    goal.position = position
    goal.velocity = velocity
    goal.position_tolerance = position_tolerance
    GT.ax_move_goal(goal)


def ax_bulk_move(moves):
    GT = get_GT()
    goals = []
    for move in moves:
        goal = ros381_tactics_py.AxMoveGoal()
        goal.id = move[0]
        goal.position = move[1]
        goal.velocity = move[2]
        goal.position_tolerance = move[3]
        goals.append(goal)

    GT.ax_bulk_move_goal(goals)


def ax_hybrid_move(id, velocity, zero_time, delta_pos):
    get_GT().ax_hybrid_move_goal(id, velocity, zero_time, delta_pos)
