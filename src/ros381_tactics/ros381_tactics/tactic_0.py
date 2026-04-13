import math
import threading
from ros381_tactics.movement import *
from ros381_tactics.get_set import *
from ros381_tactics.ax12a import *

tactic_state = 0
prev_state = -1

start_x = -1.057
start_y = 0.75
start_phi = -math.pi / 2
start_x_offset = 0.0

first_x = -1.057
first_y = 0.4
first_dir = 1

temp_x = 0.0
temp_y = 0.0
temp_dir = 0
temp_phi = 0.0

cursor_phi = 0.0

offset_x = start_x
offset_y = start_y
offset_phi = start_phi
offset_phi_tol = 0.05  # oko 3 stepena
offset_d_tol = 0.03


def load_t0():
    global start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset
    print("Tactic 0 loaded.")
    return start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset


def tactic_0():
    global tactic_state, temp_x, temp_y, temp_dir, temp_phi, first_x, first_y, first_dir, offset_x, offset_y, offset_phi, prev_state

    if prev_state != tactic_state:
        prev_state = tactic_state
        print("Tactic state = " + str(tactic_state))
    match tactic_state:
        case 0:
            move_to_xy(first_x, first_y, first_dir)
            tactic_state = 10
        case 10:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 11
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 0
        case 11:
            # T4 # move_to_xy(x=-0.35, y=0.15, dir=1)
            move_to_xy(x=-0.38, y=0.15, dir=1)
            tactic_state = 12
        case 12:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 13
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 11
        case 13:
            rotate_to_phi(phi=-math.pi * 0.5)
            tactic_state = 14
        case 14:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 15
        case 15:
            if snapshot_fsm(1, 10) < 0:
                deploy_ff(1)
                tactic_state = 20
        case 20:
            move_to_xy_unsided(x=get_GT().x_base, y=-0.0625, dir=1)
            # move_on_angle(dist=0.195, dir=1, phi=-math.pi * 0.5)
            tactic_state = 30
        case 30:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 40
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 20
        case 40:
            state, position = lift(1, -1)
            if state < 0:
                tactic_state = 50
            # TODO: ovde alt
        case 50:
            if lift_carry(1) < 0:
                tactic_state = 55
        case 55:
            if not is_mech_running():
                mechanism_thread(1)
                tactic_state = 60
        case 60:
            # T4
            # if get_side() == 1:
            #     move_to_xy(x=-0.40, y=-0.45, dir=-1)
            # else:
            #     move_to_xy(x=-0.37, y=-0.45, dir=-1)
            if get_side() == 1:
                move_to_xy(x=-0.43, y=-0.45, dir=-1)
            else:
                move_to_xy(x=-0.40, y=-0.45, dir=-1)
            tactic_state = 65
        case 65:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 70
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 60
        case 70:
            rotate_to_phi(phi=math.pi * 0.5)
            tactic_state = 72
        case 72:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 75
        case 75:
            if snapshot_fsm(-1, 10) < 0:
                deploy_ff(-1)                
                tactic_state = 76
        case 76: # 0.2375
            # move_on_angle(dist=0.2275, dir=-1, phi=math.pi * 0.5)
            move_to_xy_unsided(x=get_GT().x_base, y=-0.678, dir=-1)
            tactic_state = 77
        case 77:
            if move_success() or move_stacked():
                tactic_state = 78
        case 78:
            state, position = lift(-1, -1)
            if state < 0:
                tactic_state = 79
        case 79:
            if lift_carry(-1) < 0:
                tactic_state = 80
        case 80:
            # move_on_angle(dist=0.15, dir=-1, phi=math.pi * 0.5, v_max=0.2)
            move_to_xy_unsided(x=get_GT().x_base, y=-0.8275, dir=-1, v_max=0.2)
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
                        offset_y = get_GT().y_base
                    get_GT().update_pose(0.0, offset_y, offset_phi, 11)
                    tactic_state = 86
                else:
                    tactic_state = 90
            elif move_success() or move_failed():
                tactic_state = 90
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 80
        case 86:
            if get_update_pose_result() == -1:
                tactic_state = 95
        case 95:
            if not is_mech_running():
                mechanism_thread(-1)
                tactic_state = 96
        case 96:
            move_on_direction(dist=0.15, dir=1)
            tactic_state = 98
        case 98:
            if move_success():
                tactic_state = 100
        case 100:
            move_to_xy(x=-0.72, y=-0.42, dir=1)
            tactic_state = 101
        case 101:
            if move_success():
                tactic_state = 102
        case 102:
            rotate_to_phi(phi=1.4)
            tactic_state = 105
        case 105:
            if move_success():
                tactic_state = 110
        case 110:
            if mechanism_drop(1) < 0:
                tactic_state = 118
        case 118:
            move_on_direction(dist=0.3, dir=-1)
            tactic_state = 120
        case 120:
            if move_success():
                tactic_state = 122
        case 122:
            if mechanism_reset(1) < 0:
                tactic_state = 124
        case 124:
            direction = 0
            if get_side() == 1:
                direction = -1
            else:
                direction = 1
            move_to_xy(x=-0.2, y=-0.79, dir=direction)
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
            if get_side() == 1:
                move_cursor(dist=0.6, phi=cursor_phi)
            else:
                move_cursor(dist=0.58, phi=cursor_phi)
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
            move_to_xy(x=-1.0, y=-0.595, dir=-1)
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
                deploy_ff(1)          
                tactic_state = 240
        case 240:
            move_on_angle(dist=0.1525, dir=1, phi=math.pi)
            tactic_state = 241
        case 241:
            if move_success() or move_stacked():
                tactic_state = 242
        case 242:
            state, position = lift(1, -1)
            if state < 0:
                tactic_state = 243
        case 243:
            if lift_carry(1) < 0:
                tactic_state = 244
        case 244:
            move_on_angle(dist=0.15, dir=1, phi=math.pi, v_max=0.2)
            tactic_state = 246
        case 246:
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
                    if abs(get_GT().x_base) > 1.4375 - offset_d_tol:
                        offset_x = 1.4375
                    else:
                        offset_x = get_GT().x_base
                    get_GT().update_pose(
                        sign(get_GT().x_base) * abs(offset_x), 0.0, offset_phi, 101
                    )
                    tactic_state = 250
                else:
                    tactic_state = 270
            elif move_success():
                tactic_state = 250
        case 250:
            if get_update_pose_result() == -1:
                tactic_state = 270
        case 270:
            if not is_mech_running():
                mechanism_thread(1)
                tactic_state = 272
        case 272:
            move_on_direction(dist=0.2, dir=-1)
            tactic_state = 275
        case 275:
            if move_success() or move_stacked():
                tactic_state = 170

        case 170:
            move_to_xy(x=-1.02, y=-0.68, dir=-1)
            tactic_state = 172
        case 172:
            if move_success() or move_stacked():
                tactic_state = 175
        case 175:
            rotate_to_phi(phi=3 / 4 * math.pi)
            tactic_state = 178
        case 178:
            if move_success():
                tactic_state = 180
        case 180:
            if mechanism_drop(-1) < 0:
                tactic_state = 190
        case 190:
            move_on_direction(dist=0.1, dir=-1)
            tactic_state = 192
        case 192:
            if move_success() or move_stacked():
                tactic_state = 195
        case 195:
            move_on_direction(dist=0.3, dir=1)
            tactic_state = 200
        case 200:
            if move_success():
                tactic_state = 210
        case 210:
            if mechanism_reset(-1) < 0:
                tactic_state = 280

        case 280:
            move_to_xy(x=-1.18, y=-0.2, dir=1)
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
            if mechanism_drop(1) < 0:
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
            if get_side() == 1:
                move_to_xy(x=-1.0, y=0.2, dir=1)
            else:
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
                deploy_ff(1)          
                tactic_state = 340
        case 340:
            move_on_angle(dist=0.1525, dir=1, phi=math.pi)
            tactic_state = 341
        case 341:
            if move_success() or move_stacked():
                tactic_state = 342
        case 342:
            state, position = lift(1, -1)
            if state < 0:
                tactic_state = 343
        case 343:
            if lift_carry(1) < 0:
                tactic_state = 344
        case 344:
            move_on_angle(dist=0.16, dir=1, phi=math.pi, v_max=0.25)
            tactic_state = 346
        case 346:
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
                    if abs(get_GT().x_base) > 1.4375 - offset_d_tol:
                        offset_x = 1.4375
                    else:
                        offset_x = get_GT().x_base
                    get_GT().update_pose(
                        sign(get_GT().x_base) * abs(offset_x), 0.0, offset_phi, 101
                    )
                    tactic_state = 350
                else:
                    tactic_state = 370
            elif move_success():
                tactic_state = 350
        case 350:
            if get_update_pose_result() == -1:
                tactic_state = 370
        case 370:
            if not is_mech_running():
                mechanism_thread(1)
                tactic_state = 372
        case 372:
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
            move_to_xy(x=-0.30, y=-0.60, dir=-1)
            tactic_state = 400
        case 400:
            if move_success() or move_stacked():
                tactic_state = 410
        case 410:
            rotate_to_phi(phi=-0.25 * math.pi)
            tactic_state = 415
        case 415:
            if move_success():
                tactic_state = 420
        case 420:
            if mechanism_drop(1) < 0:
                tactic_state = 430
        case 430:
            move_on_direction(dist=0.175, dir=1)
            tactic_state = 435
        case 435:
            if move_success() or move_stacked():
                tactic_state = 440
        case 440:
            move_on_direction(dist=0.3, dir=-1)
            tactic_state = 450
        case 450:
            if move_success():
                tactic_state = 460
        case 460:
            if mechanism_reset(1) < 0:
                tactic_state = 980 # TODO: ovde da ne prekine taktiku
        case 470:
            move_to_xy(x=1.0, y=0.2, dir=-1)
            tactic_state = 475
        case 475:
            if move_success():
                tactic_state = 480
        case 480:
            rotate_to_phi(phi=0.0)
            tactic_state = 485
        case 485:
            if move_success():
                tactic_state = 490
        case 490:
            if snapshot_fsm(1, 10) < 0:
                deploy_ff(1)          
                tactic_state = 500
        case 500:
            move_on_angle(dist=0.3, dir=1, phi=0.0, v_max=0.25)
            tactic_state = 505
        case 505:
            if move_stacked():
                if get_side() == 1:
                    offset_phi = math.pi
                else:
                    offset_phi = 0.0
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
                    if abs(get_GT().x_base) > 1.4375 - offset_d_tol:
                        offset_x = 1.4375
                    else:
                        offset_x = get_GT().x_base
                    get_GT().update_pose(
                        sign(get_GT().x_base) * abs(offset_x), 0.0, offset_phi, 101
                    )
                    tactic_state = 506
                else:
                    tactic_state = 510
            elif move_success():
                tactic_state = 510
        case 506:
            if get_update_pose_result() == -1:
                tactic_state = 510
        case 510:
            state, position = lift(1, -1)
            if state < 0:
                tactic_state = 520
        case 520:
            if lift_carry(1) < 0:
                tactic_state = 530
        case 530:
            move_on_direction(dist=0.2, dir=-1)
            tactic_state = 535
        case 535:
            if move_success():
                tactic_state = 540
        case 540:
            move_to_xy(x=1.0, y=-0.5, dir= -1)
            tactic_state = 545
        case 545:
            if move_success():
                tactic_state = 980




        case 980:
            move_to_xy(0.0, -0.42, 1)
            tactic_state = 990
        case 990:
            if move_success() or move_failed()or move_stacked() or move_interrupted() :
                undeploy_ff(-1)
                tactic_state = 995
                print("Current time: " + str(get_GT().time))
        case 995:
            if get_GT().time > 95.0:
                tactic_state = 1000

        case 1000:
            move_to_xy(-1.05, -0.62, -1)
            tactic_state = 1010
        case 1010:
            if move_success():
                tactic_state = 1020
        case 1020:
            move_to_xy(-1.1, 0.78, -1)
            tactic_state = 1030
        case 1030:
            if move_success():
                tactic_state = -1

        case -1:
            print("Tactic 0 finished.")
    return tactic_state
