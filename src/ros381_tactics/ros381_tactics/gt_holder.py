_GT_instance = None


def set_GT(gt):
    global _GT_instance
    _GT_instance = gt


def get_GT():
    return _GT_instance


def get_move_result():
    return _GT_instance.move_result_
