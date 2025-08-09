from ros381_tactics.movement import *
from ros381_tactics.tactic_0 import tactic_0
from ros381_tactics.tactic_1 import tactic_1


def hello_tactics():
    print("Individual tactics python module loaded.")
    return True


tactic_state = 0
tactic_return_value = 0


def execute_tactic(GT, tactic_number, tactic_side):
    global tactic_return_value
    match tactic_number:
        case 0:
            tactic_return_value = tactic_0(GT)
        case 1:
            tactic_return_value = tactic_1(GT)
    return tactic_return_value
