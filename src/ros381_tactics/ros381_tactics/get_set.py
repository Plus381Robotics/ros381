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
    if get_side() == -1:
        return -x, get_side() * math.pi - phi
    return x, phi
