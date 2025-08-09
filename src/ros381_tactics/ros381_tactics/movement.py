from ros381_tactics.gt_holder import get_GT


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
        x=dist,
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
        x=dist,
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
        x=dist,
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
