import math

_GT_instance = None
_tactic_side = 0


def set_GT(gt):
    global _GT_instance
    _GT_instance = gt


def get_GT():
    return _GT_instance


def get_move_result():
    return _GT_instance.move_result_


def get_update_pose_result():
    return _GT_instance.update_pose_result_


def set_side(side):
    global _tactic_side
    _tactic_side = side


def get_side():
    global _tactic_side
    return _tactic_side


def sided_coords(x, phi):
    if get_side() == 1:
        return -x, get_side() * math.pi - phi
    return x, phi


_snapshot_state = 0
cf_local = [0, 0, 0, 0]
cb_local  = [0, 0, 0, 0]
cfl_x = cfl_y = cfl_phi = 9.9
cbl_x = cbl_y = cbl_phi = 9.9
_snapshot_repeat = 1
cfl_x_sum = cfl_y_sum = cfl_phi_sum = 0.0
cbl_x_sum = cbl_y_sum = cbl_phi_sum = 0.0

def snapshot_fsm(side, color, repeat):
    global _snapshot_state, cf_local, cb_local, _snapshot_repeat
    global cfl_x, cfl_y, cfl_phi
    global cbl_x, cbl_y, cbl_phi
    global cfl_x_sum, cfl_y_sum, cfl_phi_sum
    global cbl_x_sum, cbl_y_sum, cbl_phi_sum
    match _snapshot_state:
        case 0:
            if side == 1:
                if _GT_instance.cs_front_full:
                    cf_local = list(_GT_instance.crates_front)
                    cfl_x = _GT_instance.cs_front_x
                    cfl_y = _GT_instance.cs_front_y
                    cfl_phi = _GT_instance.cs_front_phi
                    _snapshot_state = 10
                else:
                    for i in range(4):
                        print("cf[" + str(i) + "] = " + str(_GT_instance.crates_front[i]))
                        if cf_local[i] != color:
                            if _GT_instance.crates_front[i] == color:
                                cf_local[i] = color
                    cfl_x_sum += _GT_instance.cs_front_x
                    cfl_y_sum += _GT_instance.cs_front_y
                    cfl_phi_sum += _GT_instance.cs_front_phi

                    if _snapshot_repeat >= repeat:
                        _snapshot_state = 10
                        cfl_x = cfl_x_sum / repeat
                        cfl_y = cfl_y_sum / repeat
                        cfl_phi = cfl_phi_sum / repeat
                    else:
                        _snapshot_repeat += 1
            else:
                if _GT_instance.cs_back_full:
                    cb_local = list(_GT_instance.crates_back)
                    cbl_x = _GT_instance.cs_back_x                    
                    cbl_y = _GT_instance.cs_back_y
                    cbl_phi = _GT_instance.cs_back_phi
                    _snapshot_state = 10
                else:
                    for i in range(4):
                        if cb_local[i] != color:
                            if _GT_instance.crates_back[i] == color:
                                cb_local[i] = color
                    cbl_x_sum += _GT_instance.cs_back_x
                    cbl_y_sum += _GT_instance.cs_back_y
                    cbl_phi_sum += _GT_instance.cs_back_phi

                    if _snapshot_repeat >= repeat:
                        _snapshot_state = 10
                        cbl_x = cbl_x_sum / repeat
                        cbl_y = cbl_y_sum / repeat
                        cbl_phi = cbl_phi_sum / repeat
                    else:
                        _snapshot_repeat += 1
        case 10:
            if side == 1:
                print("Snapshot front:", cf_local, cfl_x, cfl_y, cfl_phi)
            else:
                print("Snapsho back:", cb_local, cbl_x, cbl_y, cbl_phi)  
            cfl_x_sum = 0.0
            cfl_y_sum = 0.0
            cfl_phi_sum = 0.0
            cbl_x_sum = 0.0
            cbl_y_sum = 0.0
            cbl_phi_sum = 0.0
            _snapshot_state = 0
            _snapshot_repeat = 1
            return -1
    return _snapshot_state