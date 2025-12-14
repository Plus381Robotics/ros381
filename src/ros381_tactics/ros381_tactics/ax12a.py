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
            ax_hybrid_move(4, 1000, 0.2, 200)
            init_state = 1
        case 1:
            if get_ax_hybrid_move_result() < 0:
                init_state = 2
        case 2:
            ax_hybrid_move(3, 1000, 0.2, -200)
            init_state = 3
        case 3:
            if get_ax_hybrid_move_result() < 0:
                init_state = 4
        case 4:
            ax_move(2, 0, 300, 50)
            init_state = 5
        case 5:
            if get_ax_move_result() < 0:
                init_state = 6
        case 6:

            ax_move(1, 1023, 300, 50)
            init_state = 7
        case 7:
            if get_ax_move_result() < 0:
                init_state = 8
        case 8:
            ax_bulk_move(
                [
                    (1, 511, 1000, 100),
                    (2, 511, 1000, 100),
                    (3, 511, 1000, 100),
                    (4, 511, 1000, 100),
                ]
            )
            init_state = 9
        case 9:
            if get_ax_bulk_move_result() < 0:
                init_state = -1
        case -1:
            return True
    return False


def get_ax_move_result():
    return get_GT().ax_move_result_


def get_ax_bulk_move_result():
    return get_GT().ax_bulk_move_result_


def get_ax_hybrid_move_result():
    return get_GT().ax_hybrid_move_result_


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
