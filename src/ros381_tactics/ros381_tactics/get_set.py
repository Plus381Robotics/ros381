_GT_instance = None


def set_GT(gt):
    global _GT_instance
    _GT_instance = gt


def get_GT():
    return _GT_instance


def get_move_result():
    return _GT_instance.move_result_


def get_update_pose_result():
    return _GT_instance.update_pose_result_
