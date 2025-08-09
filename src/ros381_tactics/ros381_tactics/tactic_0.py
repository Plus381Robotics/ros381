from ros381_tactics.movement import *


tactic_state = 0
tactic_return_value = 0


def load_t0(tactic_side):
    print("Tactic 0 loaded.")
    return -1.0, 0.5, 1


def tactic_0(GT, tactic_side):
    global tactic_state, tactic_return_value

    match tactic_state:
        case 0:
            move_on_direction(GT, 1.0, 1)
            tactic_state = 4
        case 4:
            if GT.move_result_ == -1:
                tactic_state = 5
        case 5:
            move_on_angle(GT, 1.0, 1, -1.57)
            tactic_state = 6
        case 6:
            if GT.move_result_ == -1:
                tactic_state = 7
        case 7:
            move_on_angle(GT, 2.0, -1, 3.14159, v_max=0.25, stop_coeff_w=10.0)
            tactic_state = 8
        case 8:
            if GT.move_result_ == -1:
                tactic_state = 9
        case 9:
            move_to_xy(GT, 0.0, 0.0, 1, ang_tol_perc=10.0, stop_coeff_v=10.0)
            tactic_state = 10
        case 10:
            if GT.move_result_ == -1:
                tactic_state = 11
        case 11:
            print("Tactic 0 finished.")
            tactic_return_value = -1
    return tactic_return_value
