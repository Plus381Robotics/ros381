from ros381_tactics.get_set import *
import math

current_retry = 0
retry_fsm_state = 0
saved_return_state = 0
saved_exit_state = 0
saved_number_of_retries = 0
saved_distance = 0.0
saved_direction = 0


def _send_goal(
    GT,
    type_,
    x=0.0,
    y=0.0,
    phi=0.0,
    dir=0,
    v_max=99.0,
    w_max=99.0,
    d_tol_perc=1.0,
    ang_tol_perc=1.0,
    start_coeff_v=1.0,
    start_coeff_w=1.0,
    stop_coeff_v=1.0,
    stop_coeff_w=1.0,
):
    sided_x, sided_phi = sided_coords(x, phi)
    GT.send_goal(
        type_,
        sided_x,
        y,
        sided_phi,
        dir,
        v_max,
        w_max,
        d_tol_perc,
        ang_tol_perc,
        start_coeff_v,
        start_coeff_w,
        stop_coeff_v,
        stop_coeff_w,
    )


def set_retry(direction, number_of_retries, return_state, exit_state, distance=0.1):
    global saved_number_of_retries, saved_distance, saved_return_state, saved_exit_state, saved_direction
    saved_direction = direction
    saved_number_of_retries = number_of_retries
    saved_return_state = return_state
    saved_exit_state = exit_state
    saved_distance = distance
    return -10


def exec_retry():
    global current_retry, retry_fsm_state, saved_number_of_retries, saved_distance, saved_return_state, saved_exit_state, saved_direction
    retval = -10
    match retry_fsm_state:
        case 0:
            current_retry += 1
            print(f"Retry {current_retry} of {saved_number_of_retries}...")
            move_on_direction(saved_distance, saved_direction)
            retry_fsm_state = 1
            pass
        case 1:
            if get_move_result() < 0:
                retry_fsm_state = 2
            pass
        case 2:
            if current_retry >= saved_number_of_retries:
                retval = saved_exit_state
                current_retry = 0
                saved_number_of_retries = 0
                saved_distance = 0.0
                saved_direction = 0
                saved_return_state = 0
                saved_exit_state = 0
            else:
                retval = saved_return_state
            retry_fsm_state = 0
    return retval


def move_success():
    if get_move_result() == -1:
        return True
    return False


def move_failed():
    if get_move_result() == -2:
        return True
    return False


def move_stacked():
    if get_move_result() == -3:
        return True
    return False


def move_interrupted():
    if get_move_result() == -4:
        return True
    return False


def rotate_to_phi(
    phi, w_max=99.0, ang_tol_perc=1.0, start_coeff_w=1.0, stop_coeff_w=1.0
):
    _send_goal(
        get_GT(),
        -1,
        phi=phi,
        w_max=w_max,
        ang_tol_perc=ang_tol_perc,
        start_coeff_w=start_coeff_w,
        stop_coeff_w=stop_coeff_w,
    )


def rotate_to_xy(
    x, y, dir, w_max=99.0, ang_tol_perc=1.0, start_coeff_w=1.0, stop_coeff_w=1.0
):
    _send_goal(
        get_GT(),
        -2,
        x=x,
        y=y,
        dir=dir,
        w_max=w_max,
        ang_tol_perc=ang_tol_perc,
        start_coeff_w=start_coeff_w,
        stop_coeff_w=stop_coeff_w,
    )


def move_to_xy(
    x,
    y,
    dir,
    v_max=99.0,
    w_max=99.0,
    d_tol_perc=1.0,
    ang_tol_perc=1.0,
    start_coeff_v=1.0,
    start_coeff_w=1.0,
    stop_coeff_v=1.0,
    stop_coeff_w=1.0,
):
    _send_goal(
        get_GT(),
        1,
        x=x,
        y=y,
        dir=dir,
        v_max=v_max,
        w_max=w_max,
        d_tol_perc=d_tol_perc,
        ang_tol_perc=ang_tol_perc,
        start_coeff_v=start_coeff_v,
        start_coeff_w=start_coeff_w,
        stop_coeff_v=stop_coeff_v,
        stop_coeff_w=stop_coeff_w,
    )


def move_on_direction(
    dist,
    dir,
    v_max=99.0,
    d_tol_perc=1.0,
    start_coeff_v=1.0,
    stop_coeff_v=1.0,
):
    _send_goal(
        get_GT(),
        2,
        y=dist,
        dir=dir,
        v_max=v_max,
        d_tol_perc=d_tol_perc,
        start_coeff_v=start_coeff_v,
        stop_coeff_v=stop_coeff_v,
    )


def move_on_direction_snapped(
    dist,
    dir,
    snap_phi,
    v_max=99.0,
    w_max=99.0,
    d_tol_perc=1.0,
    ang_tol_perc=1.0,
    start_coeff_v=1.0,
    start_coeff_w=1.0,
    stop_coeff_v=1.0,
    stop_coeff_w=1.0,
):
    _send_goal(
        get_GT(),
        3,
        y=dist,
        dir=dir,
        phi=snap_phi,
        v_max=v_max,
        w_max=w_max,
        d_tol_perc=d_tol_perc,
        ang_tol_perc=ang_tol_perc,
        start_coeff_v=start_coeff_v,
        start_coeff_w=start_coeff_w,
        stop_coeff_v=stop_coeff_v,
        stop_coeff_w=stop_coeff_w,
    )


def move_on_angle(
    dist,
    dir,
    phi,
    v_max=99.0,
    w_max=99.0,
    d_tol_perc=1.0,
    ang_tol_perc=1.0,
    start_coeff_v=1.0,
    start_coeff_w=1.0,
    stop_coeff_v=1.0,
    stop_coeff_w=1.0,
):
    _send_goal(
        get_GT(),
        4,
        y=dist,
        dir=dir,
        phi=phi,
        v_max=v_max,
        w_max=w_max,
        d_tol_perc=d_tol_perc,
        ang_tol_perc=ang_tol_perc,
        start_coeff_v=start_coeff_v,
        start_coeff_w=start_coeff_w,
        stop_coeff_v=stop_coeff_v,
        stop_coeff_w=stop_coeff_w,
    )


def _send_goal_unsided(
    GT,
    type_,
    x=0.0,
    y=0.0,
    phi=0.0,
    dir=0,
    v_max=99.0,
    w_max=99.0,
    d_tol_perc=1.0,
    ang_tol_perc=1.0,
    start_coeff_v=1.0,
    start_coeff_w=1.0,
    stop_coeff_v=1.0,
    stop_coeff_w=1.0,
):
    GT.send_goal(
        type_,
        x,
        y,
        phi,
        dir,
        v_max,
        w_max,
        d_tol_perc,
        ang_tol_perc,
        start_coeff_v,
        start_coeff_w,
        stop_coeff_v,
        stop_coeff_w,
    )


def rotate_to_phi_unsided(
    phi, w_max=99.0, ang_tol_perc=1.0, start_coeff_w=1.0, stop_coeff_w=1.0
):
    _send_goal_unsided(
        get_GT(),
        -1,
        phi=phi,
        w_max=w_max,
        ang_tol_perc=ang_tol_perc,
        start_coeff_w=start_coeff_w,
        stop_coeff_w=stop_coeff_w,
    )


def move_on_angle_unsided(
    dist,
    dir,
    phi,
    v_max=99.0,
    w_max=99.0,
    d_tol_perc=1.0,
    ang_tol_perc=1.0,
    start_coeff_v=1.0,
    start_coeff_w=1.0,
    stop_coeff_v=1.0,
    stop_coeff_w=1.0,
):
    _send_goal_unsided(
        get_GT(),
        4,
        y=dist,
        dir=dir,
        phi=phi,
        v_max=v_max,
        w_max=w_max,
        d_tol_perc=d_tol_perc,
        ang_tol_perc=ang_tol_perc,
        start_coeff_v=start_coeff_v,
        start_coeff_w=start_coeff_w,
        stop_coeff_v=stop_coeff_v,
        stop_coeff_w=stop_coeff_w,
    )


def move_cursor(
    dist,
    phi,
    v_max=99.0,
    w_max=99.0,
    d_tol_perc=1.0,
    ang_tol_perc=1.0,
    start_coeff_v=1.0,
    start_coeff_w=1.0,
    stop_coeff_v=1.0,
    stop_coeff_w=1.0,
):
 direction = 0
 if get_side() == 1:
     direction = 1
 else:
     direction = -1
 _send_goal_unsided(
        get_GT(),
        4,
        y=dist,
        dir=direction,
        phi=phi,
        v_max=v_max,
        w_max=w_max,
        d_tol_perc=d_tol_perc,
        ang_tol_perc=ang_tol_perc,
        start_coeff_v=start_coeff_v,
        start_coeff_w=start_coeff_w,
        stop_coeff_v=stop_coeff_v,
        stop_coeff_w=stop_coeff_w,
    )

def move_cursor_reverse(
    dist,
    phi,
    v_max=99.0,
    w_max=99.0,
    d_tol_perc=1.0,
    ang_tol_perc=1.0,
    start_coeff_v=1.0,
    start_coeff_w=1.0,
    stop_coeff_v=1.0,
    stop_coeff_w=1.0,
):
 direction = 0
 if get_side() == 1:
     direction = -1
 else:
     direction = 1
 _send_goal_unsided(
        get_GT(),
        4,
        y=dist,
        dir=direction,
        phi=phi,
        v_max=v_max,
        w_max=w_max,
        d_tol_perc=d_tol_perc,
        ang_tol_perc=ang_tol_perc,
        start_coeff_v=start_coeff_v,
        start_coeff_w=start_coeff_w,
        stop_coeff_v=stop_coeff_v,
        stop_coeff_w=stop_coeff_w,
    )