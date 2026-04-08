import math
from ros381_tactics.movement import *
from ros381_tactics.get_set import *
from ros381_tactics.ax12a import *

tactic_state = 0

start_x = -1.057
start_y = 0.75
start_phi = -math.pi / 2

first_x = -1.057
first_y = 0.35
first_dir = 1

temp_x = 0.0
temp_y = 0.0
temp_dir = 0
temp_phi = 0.0

cursor_phi = 0.0

offset_x = start_x
offset_y = start_y
offset_phi = start_phi
offset_phi_tol = 0.07  # oko 4 stepena
offset_d_tol = 0.075


def load_t4():
    global start_x, start_y, start_phi, first_x, first_y, first_dir
    print("Tactic 4 loaded.")
    return start_x, start_y, start_phi, first_x, first_y, first_dir


def tactic_4():
    global tactic_state, temp_x, temp_y, temp_dir, temp_phi, first_x, first_y, first_dir, offset_x, offset_y, offset_phi

    match tactic_state:
        case 0:
            move_to_xy(first_x, first_y, first_dir)
            tactic_state = 10
        case 10:
            if move_success():
                tactic_state = 11
        case 11:
            move_to_xy(x=-0.35, y=0.15, dir=1)
            tactic_state = 12
        case 12:
            if move_success():
                tactic_state = 13
        case 13:
            rotate_to_phi(phi=-math.pi * 0.5)
            tactic_state = 14
        case 14:
            if move_success():
                tactic_state = 15
        case 15:
            if snapshot_fsm(1, 10) < 0:
                tactic_state = 20
        case 20:
            move_on_angle(dist=0.30, dir=1, phi=-math.pi * 0.5, v_max=0.2)
            tactic_state = 30
        case 30:
            if move_success():
                tactic_state = 40
        case 40:
            state, position = lift(1, -1)
            if state < 0:
                tactic_state = 50
        case 50:
            if lift_carry(1) < 0:
                tactic_state = 60
        case 60:
            move_to_xy(x=-0.40, y=-0.5, dir=-1)
            tactic_state = 65
        case 65:
            if move_success():
                tactic_state = 70
        case 70:
            rotate_to_phi(phi=math.pi * 0.5)
            tactic_state = 75
        case 75:
            if move_success():
                tactic_state = 77
        case 77:
            if snapshot_fsm(-1, 10) < 0:
                tactic_state = 80
        case 80:
            move_on_angle(dist=0.3, dir=-1, phi=math.pi * 0.5, v_max=0.2)
            tactic_state = 85
        case 85:
            if move_stacked():
                offset_phi = math.pi * 0.5
                print(
                    "Coordinates are (x, y, phi): "
                    + str(get_GT().x_base)
                    + ", "
                    + str(get_GT().y_base)
                    + ", "
                    + str(get_GT().phi_base)
                )
                if (
                    get_GT().phi_base < offset_phi + offset_phi_tol
                    and get_GT().phi_base > offset_phi - offset_phi_tol
                ):
                    if get_GT().y_base < -0.9375 + offset_d_tol:
                        offset_y = -0.9375
                    else:
                        offset_y = -0.7875
                    get_GT().update_pose(0.0, offset_y, offset_phi, 11)
                    tactic_state = 86
                else:
                    tactic_state = 90
            elif move_success():
                tactic_state = 90
        case 86:
            if get_update_pose_result() == -1:
                tactic_state = 90
        case 90:
            state, position = lift(-1, -1)
            if state < 0:
                tactic_state = 92
        case 92:
            if lift_carry(-1) < 0:
                tactic_state = 95
        case 95:
            move_on_direction(dist=0.15, dir=1)
            tactic_state = 98
        case 98:
            if move_success():
                tactic_state = 100
        case 100:
            move_to_xy(x=-0.4, y=-0.34, dir=1)
            tactic_state = 101
        case 101:
            if move_success():
                tactic_state = 102
        case 102:
            rotate_to_xy(x=-0.1, y=-0.65, dir=-1)
            tactic_state = 105
        case 105:
            if move_success():
                tactic_state = 110
        case 110:
            if mechanism(1) < 0:
                tactic_state = 112
        case 112:
            rotate_to_xy(x=-0.1, y=-0.65, dir=-1)
            tactic_state = 115
        case 115:
            if move_success():
                tactic_state = 118
        case 118:
            if mechanism_reset(1) < 0:
                tactic_state = 122
        case 122:
            rotate_to_phi_unsided(phi=cursor_phi, w_max=3.14)
            tactic_state = 123
        case 123:
            if move_success():
                tactic_state = 124
        case 124:
            direction = 0
            if get_side() == 1:
                direction = -1
            else:
                direction = 1
            move_to_xy(x=-0.2, y=-0.8, dir=direction)
            tactic_state = 125
        case 125:
            if move_success():
                tactic_state = 130
        case 130:
            rotate_to_phi_unsided(phi=cursor_phi, w_max=3.14)
            tactic_state = 135
        case 135:
            if move_success():
                tactic_state = 138
        case 138:
            if cursor(-1) < 0:
                tactic_state = 140
        case 140:
            move_cursor(dist=0.6, phi=cursor_phi)
            tactic_state = 145
        case 145:
            if move_success() or move_failed() or move_interrupted() or move_stacked():
                tactic_state = 150
        case 150:
            if cursor(1) < 0:
                tactic_state = 160
        case 160:
            move_cursor(dist=0.2, phi=cursor_phi)
            tactic_state = 165
        case 165:
            if move_success():
                tactic_state = 212

        case 212:
            move_to_xy(x=-1.0, y=-0.6, dir=-1)
            tactic_state = 215
        case 215:
            if move_success():
                tactic_state = 220
        case 220:
            rotate_to_phi(phi=math.pi)
            tactic_state = 225
        case 225:
            if move_success():
                tactic_state = 230
        case 230:
            if snapshot_fsm(1, 10) < 0:
                tactic_state = 240
        case 240:
            move_on_angle(dist=0.3, dir=1, phi=math.pi, v_max=0.2)
            tactic_state = 245
        case 245:
            if move_stacked():
                if get_side() == 1:
                    offset_phi = 0.0
                else:
                    offset_phi = math.pi
                print(
                    "Coordinates are (x, y, phi): "
                    + str(get_GT().x_base)
                    + ", "
                    + str(get_GT().y_base)
                    + ", "
                    + str(get_GT().phi_base)
                )
                if (
                    get_GT().phi_base < offset_phi + offset_phi_tol
                    and get_GT().phi_base > offset_phi - offset_phi_tol
                ):
                    if get_GT().x_base < -1.4375 + offset_d_tol:
                        offset_x = 1.4375
                    else:
                        offset_x = 1.2875
                    get_GT().update_pose(
                        sign(get_GT().x_base) * offset_x, 0.0, offset_phi, 101
                    )
                    tactic_state = 246
                else:
                    tactic_state = 250
            elif move_success():
                tactic_state = 250
        case 246:
            if get_update_pose_result() == -1:
                tactic_state = 250
        case 250:
            state, position = lift(1, -1)
            if state < 0:
                tactic_state = 260
        case 260:
            if lift_carry(1) < 0:
                tactic_state = 270
        case 270:
            move_on_direction(dist=0.3, dir=-1)
            tactic_state = 275
        case 275:
            if move_success() or move_stacked():
                tactic_state = 170

        case 170:
            move_to_xy(x=-0.92, y=-0.7, dir=-1)
            tactic_state = 175
        case 175:
            if move_success() or move_stacked():
                tactic_state = 180
        case 180:
            if mechanism(-1) < 0:
                tactic_state = 190
        case 190:
            move_on_direction(dist=0.3, dir=1)
            tactic_state = 200
        case 200:
            if move_success():
                tactic_state = 210
        case 210:
            if mechanism_reset(-1) < 0:
                tactic_state = 280

        case 280:
            move_to_xy(x=-1.2, y=-0.2, dir=1)
            tactic_state = 285
        case 285:
            if move_success() or move_stacked():
                tactic_state = 286
        case 286:
            rotate_to_phi(phi=math.pi)
            tactic_state = 288
        case 288:
            if move_success():
                tactic_state = 290
        case 290:
            if mechanism(1) < 0:
                tactic_state = 292
        case 292:
            move_on_direction(dist=0.2, dir=-1)
            tactic_state = 295
        case 295:
            if move_success():
                tactic_state = 298
        case 298:
            if mechanism_reset(1) < 0:
                tactic_state = 310
        case 310:
            move_to_xy(x=-1.0, y=0.2, dir=1)
            tactic_state = 315
        case 315:
            if move_success():
                tactic_state = 320
        case 320:
            rotate_to_phi(phi=math.pi)
            tactic_state = 325
        case 325:
            if move_success():
                tactic_state = 330
        case 330:
            if snapshot_fsm(1, 10) < 0:
                tactic_state = 340
        case 340:
            move_on_angle(dist=0.3, dir=1, phi=math.pi, v_max=0.2)
            tactic_state = 345
        case 345:
            if move_stacked():
                if get_side() == 1:
                    offset_phi = 0.0
                else:
                    offset_phi = math.pi
                print(
                    "Coordinates are (x, y, phi): "
                    + str(get_GT().x_base)
                    + ", "
                    + str(get_GT().y_base)
                    + ", "
                    + str(get_GT().phi_base)
                )
                if (
                    get_GT().phi_base < offset_phi + offset_phi_tol
                    and get_GT().phi_base > offset_phi - offset_phi_tol
                ):
                    if get_GT().x_base < -1.4375 + offset_d_tol:
                        offset_x = 1.4375
                    else:
                        offset_x = 1.2875
                    get_GT().update_pose(
                        sign(get_GT().x_base) * offset_x, 0.0, offset_phi, 101
                    )
                    tactic_state = 346
                else:
                    tactic_state = 350
            elif move_success():
                tactic_state = 350
        case 346:
            if get_update_pose_result() == -1:
                tactic_state = 350
        case 350:
            state, position = lift(1, -1)
            if state < 0:
                tactic_state = 360
        case 360:
            if lift_carry(1) < 0:
                tactic_state = 370
        case 370:
            move_on_direction(dist=0.2, dir=-1)
            tactic_state = 375
        case 375:
            if move_success() or move_stacked():
                tactic_state = 380
        case 380:
            move_to_xy(-1.0, -0.5, -1)
            tactic_state = 385
        case 385:
            if move_success():
                tactic_state = 390
        case 390:
            move_to_xy(-1.2, -0.7, -1)
            tactic_state = 395
        case 395:
            if move_success():
                tactic_state = 1000
                # Ovde treba da saceka vreme pa ode kuci

        case 1000:
            move_to_xy(-1.0, -0.5, 1)
            tactic_state = 1010
        case 1010:
            if move_success():
                tactic_state = 1015
        case 1015:
            if reset_to_undeployed(-1) < 0:
                tactic_state = 1020
        case 1020:
            move_to_xy(-1.2, 0.75, -1)
            tactic_state = 1030
        case 1030:
            if move_success():
                tactic_state = -1

        case -1:
            print("Tactic 4 finished.")
    return tactic_state
