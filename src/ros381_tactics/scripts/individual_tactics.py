def hello_tactics():
    print("Individual tactics python module loaded.")
    return True


def tactic_0(node):
    print("Tactic 0 loaded.")
    node.send_goal(1, 1.0, 0.5, 0.0, 1, 2.0, 12.6, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
