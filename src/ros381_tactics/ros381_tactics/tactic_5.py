import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *

tactic_state = 0

start_x = -1.06
start_y = 0.735
start_phi = -math.pi / 2
start_x_offset = 0.02

first_x = -1.06
first_y = 0.4
first_dir = 1

temp_x = 0.0
temp_y = 0.0
temp_dir = 0
temp_phi = 0.0

prev_state = 0


def load_t5():
    global start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset
    print("Tactic 5 loaded.")
    return start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset


def tactic_5():
    global tactic_state, temp_x, temp_y, temp_dir, temp_phi, first_x, first_y, first_dir, prev_state

    if prev_state != tactic_state:
        print("State: " + str(tactic_state))
        prev_state = tactic_state

    match tactic_state:
        case 0:
            move_on_direction(dist=0.4, dir=1)
            tactic_state = 2
        case 2:
            if move_success():
                tactic_state = 10
        case 10:
            move_to_xy(x=0.35, y=0.15, dir=1)
            tactic_state = 12
        case 12:
            if move_success():
                tactic_state = 20
            elif move_interrupted():
                tactic_state = 20
        case 20:
            move_to_xy(x=0.35, y=-0.0625, dir=1)
            tactic_state = 22
        case 22:
            if move_success():
                tactic_state = 30
        case 30:
            move_to_xy(x=0.4, y=-0.45, dir=1)
            tactic_state = 32
        case 32:
            if move_success():
                tactic_state = 40
        case 40:
            move_on_angle(dist=0.35, dir=-1, phi=math.pi / 2)
            tactic_state = 42
        case 42:
            if move_success() or move_stacked():
                tactic_state = 50
        case 50:
            move_on_direction(dist=0.15, dir=1)
            tactic_state = 52
        case 52:
            if move_success():
                tactic_state = 55
        case 55:
            move_to_xy(x=-0.5, y=-0.5, dir=1)
            tactic_state = 57
        case 57:
            if move_success():
                tactic_state = 60
        case 60:
            move_to_xy(x=-1.2, y=-0.2, dir=1)
            tactic_state = 62
        case 62:
            if move_success():
                tactic_state = 70
        case 70:
            move_on_angle(dist=0.2, dir=1, phi=math.pi)
            tactic_state = 72
        case 72:
            if move_success() or move_stacked():
                tactic_state = 75
        case 75:
            move_on_direction(dist=0.15, dir=-1)
            tactic_state = 77
        case 77:
            if move_success():
                tactic_state = 80
        case 80:
            move_to_xy(x=-0.735, y=-0.25, dir=-1)
            tactic_state = 82
        case 82:
            if move_success():
                tactic_state = 90
        case 90:
            temp_x = -1.0
            temp_y = -0.6
            temp_dir = 1
            rotate_to_xy(x=temp_x, y=temp_y, dir=temp_dir)
            tactic_state = 92
        case 92:
            if move_success():
                tactic_state = 100
        case 100:
            move_to_xy(x=temp_x, y=temp_y, dir=temp_dir)
            tactic_state = 102
        case 102:
            if move_success():
                tactic_state = 110
        case 110:
            move_on_angle(dist=0.3, dir=1, phi=math.pi)
            tactic_state = 112
        case 112:
            if move_success() or move_stacked():
                tactic_state = 115
        case 115:
            move_on_direction(dist=0.15, dir=-1)
            tactic_state = 117
        case 117:
            if move_success():
                tactic_state = 120
        case 120:
            move_to_xy(x=-0.4, y=-0.5, dir=-1)
            tactic_state = 122
        case 122:
            if move_success():
                tactic_state = 130
        case 130:
            move_on_angle(dist=0.3, dir=-1, phi=math.pi / 2)
            tactic_state = 132
        case 132:
            if move_success() or move_stacked():
                tactic_state = 140
        case 140:
            move_on_direction(dist=0.2, dir=1)
            tactic_state = 142
        case 142:
            if move_success():
                tactic_state = 150
        case 150:
            move_to_xy(x=-0.1, y=-0.8, dir=-1)
            tactic_state = 152
        case 152:
            if move_success():
                tactic_state = 160
        case 160:
            move_on_direction(dist=0.3, dir=1)
            tactic_state = 162
        case 162:
            if move_success():
                tactic_state = 170
        case 170:
            move_to_xy(x=-1.2, y=-0.8, dir=-1)
            tactic_state = 172
        case 172:
            if move_success():
                tactic_state = 180
        case 180:
            move_on_angle(dist=0.15, dir=-1, phi=0.0)
            tactic_state = 182
        case 182:
            if move_success() or move_stacked():
                tactic_state = 190
        case 190:
            move_to_xy(x=-0.8, y=-0.8, dir=1)
            tactic_state = 192
        case 192:
            if move_success():
                tactic_state = 200
        case 200:
            move_on_angle(dist=0.25, dir=1, phi=math.pi / 2)
            tactic_state = 202
        case 202:
            if move_success():
                tactic_state = 210
        case 210:
            move_on_angle(dist=0.2, dir=1, phi=-math.pi / 2)
            tactic_state = 212
        case 212:
            if move_success() or move_stacked():
                tactic_state = 220
        case 220:
            move_on_direction(dist=0.2, dir=-1)
            tactic_state = 222
        case 222:
            if move_success():
                tactic_state = 230
        case 230:
            move_to_xy(x=-0.35, y=-0.5, dir=-1)
            tactic_state = 232
        case 232:
            if move_success():
                tactic_state = 240
        case 240:
            move_to_xy(x=-0.35, y=-0.3375, dir=-1)
            tactic_state = 242
        case 242:
            if move_success():
                tactic_state = 250
        case 250:
            move_to_xy(x=-0.25, y=0.2, dir=-1)
            tactic_state = 252
        case 252:
            if move_success():
                tactic_state = 260
        case 260:
            move_on_angle(dist=0.2, dir=-1, phi=-math.pi / 2)
            tactic_state = 262
        case 262:
            if move_success() or move_stacked():
                tactic_state = 270
        case 270:
            move_on_direction(dist=0.2, dir=1)
            tactic_state = 272
        case 272:
            if move_success():
                tactic_state = 280
        case 280:
            move_to_xy(x=-1.1, y=0.2, dir=-1)
            tactic_state = 282
        case 282:
            if move_success():
                tactic_state = 290
        case 290:
            move_on_angle(dist=0.2, dir=-1, phi=0.0)
            tactic_state = 292
        case 292:
            if move_success() or move_stacked():
                tactic_state = 300
        case 300:
            move_to_xy(x=-0.7, y=0.2, dir=1)
            tactic_state = 302
        case 302:
            if move_success():
                tactic_state = 310
        case 310:
            move_on_angle(dist=0.2, dir=1, phi=math.pi/2)
            tactic_state = 312
        case 312:
            if move_success() or move_stacked():
                tactic_state = 320
        case 320:
            move_to_xy(x=-0.7, y=0.35, dir=-1)
            tactic_state = 322
        case 322:
            if move_success():
                tactic_state = 330
        case 330:
            move_to_xy(x=-0.0625, y=0.35, dir=-1)
            tactic_state = 332
        case 332:
            if move_success():
                tactic_state = 340
        case 340:
            move_to_xy(x=-0.3875, y=0.35, dir=1)
            tactic_state = 342
        case 342:
            if move_success():
                tactic_state = 350
        case 350:
            move_to_xy(x=-1.06, y=0.35, dir=1)
            tactic_state = 352
        case 352:
            if move_success():
                tactic_state = 360
        case 360:
            move_on_angle(dist=0.75, dir=1, phi=math.pi/2, v_max=0.5)
            tactic_state=362
        case 362:
            if move_success() or move_stacked():
                tactic_state = -1
        case -1:
            print("Tactic 5 finished.")
    return tactic_state
