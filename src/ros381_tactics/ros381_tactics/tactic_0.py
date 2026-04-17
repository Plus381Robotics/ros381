import math
import threading
import time
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
offset_phi_tol = 0.034  # oko 2 stepena
offset_d_tol = 0.02

flag_skip_back = False
flag_skip_front_1 = False
flag_skip_front_2 = False
flag_back_skipped = False

def load_t0():
    global start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset
    print("Tactic 0 loaded.")
    return start_x, start_y, start_phi, first_x, first_y, first_dir, start_x_offset


def tactic_0():
    global tactic_state, temp_x, temp_y, temp_dir, temp_phi, first_x, first_y, first_dir, offset_x, offset_y, offset_phi, prev_state, flag_skip_back, flag_back_skipped, flag_skip_front_1, flag_skip_front_2

    if prev_state != tactic_state:
        prev_state = tactic_state
        print("---------------------------------")
        print("Tactic state = " + str(tactic_state))
        print("Current time = " + str(get_GT().time))

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
        # Prilazna za 3
        case 11:
            if get_side() == 1: # plava
                move_to_xy(x=-0.39, y=0.15, dir=1)
            else: #zuta
                move_to_xy(x=-0.36, y=0.15, dir=1)
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
        # Hvata 3
        case 20:
            move_to_xy_unsided(x=get_GT().x_base, y=-0.055, dir=1)
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
        case 50:
            if lift_carry(1) < 0:
                tactic_state = 55
        case 55:
            if not is_mech_running():
                mechanism_thread(1)
                tactic_state = 60
        # Prilazna za 4
        case 60:
            if get_side() == 1: # plava
                move_to_xy(x=-0.425, y=-0.5, dir=-1)
            else: # zuta
                move_to_xy(x=-0.375, y=-0.45, dir=-1)
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
                if cb_empty():
                    tactic_state = 100
                    flag_skip_back = True
                    flag_back_skipped = True
                else:
                    deploy_ff(-1)
                    tactic_state = 76
        # Hvata 4
        case 76: # 0.2375
            # move_on_angle(dist=0.2275, dir=-1, phi=math.pi * 0.5)
            if get_side() == 1:
                move_to_xy_unsided(x=get_GT().x_base, y=-0.688, dir=-1)
            else:
                move_to_xy_unsided(x=get_GT().x_base, y=-0.678, dir=-1)
            tactic_state = 77
        case 77:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 78
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 76
        case 78:
            state, position = lift(-1, -1)
            if state < 0:
                tactic_state = 79
        case 79:
            if lift_carry(-1) < 0:
                tactic_state = 80
        # Stackuje se u zid
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
                    if get_GT().y_base < -0.7875 + offset_d_tol:
                        offset_y = -0.7875
                    else:
                        offset_y = get_GT().y_base
                    get_GT().update_pose(0.0, offset_y, offset_phi, 11)
                    tactic_state = 86
                else:
                    tactic_state = 90
            elif move_success() or move_failed() or move_interrupted():
                tactic_state = 90
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
            if move_success() or move_stacked() or move_failed():
                tactic_state = 100
            if move_interrupted():
                if get_GT().y_base > -0.87:
                    tactic_state = 100
                else:
                    time.sleep(0.1)
                    tactic_state = 96
        # Ostavlja u 2
        case 100:
            move_to_xy(x=-0.75, y=-0.31, dir=1)
            tactic_state = 101
        case 101:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 102
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 100
        case 102:
            rotate_to_phi(phi=1.2)
            tactic_state = 105
        case 105:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 110
        case 110:
            if mechanism_drop(1) < 0:
                tactic_state = 118
        case 118:
            move_to_xy(x=-0.9, y=-0.62, dir= -1)
            tactic_state = 120
        case 120:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 122
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 118
        case 122:
            if mechanism_reset(1) < 0:
                tactic_state = 124
        case 124:
            # Prilazna za cursor
            direction = 0
            if get_side() == 1:
                direction = -1
                move_to_xy(x=-0.255, y=-0.79, dir=direction)
            else:
                direction = 1
                move_to_xy(x=-0.2, y=-0.79, dir=direction)
            tactic_state = 125
        case 125:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 130
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 124
        case 130:
            rotate_to_phi_unsided(phi=cursor_phi)
            tactic_state = 135
        case 135:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 138
        case 138:
            if cursor(-1) < 0:
                tactic_state = 140
        case 140:
            if get_side() == 1:
                move_cursor(dist=0.56, phi=cursor_phi)
            else:
                move_cursor(dist=0.57, phi=cursor_phi)
            tactic_state = 145
        case 145:
            if move_success() or move_failed() or move_interrupted() or move_stacked():
                tactic_state = 150
        case 150:
            if cursor(1) < 0:
                tactic_state = 160
        case 160:
            move_cursor_reverse(dist=0.2, phi=cursor_phi)
            tactic_state = 165
        case 165:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 170
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 160 # TODO
        case 166:
            move_cursor(dist=0.16, phi=cursor_phi)
            tactic_state = 167
        case 167:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 170
        # Prilazna za 2
        case 170:
            if get_side() == 1:
                move_to_xy(x=-0.9, y=-0.605, dir=1)
            else:
                move_to_xy(x=-0.9, y=-0.615, dir=-1)
            tactic_state = 175
        case 175:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 180
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 170
        case 180:
            rotate_to_phi(phi=math.pi)
            tactic_state = 190
        case 190:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 195
        case 195:
            if snapshot_fsm(1, 10) < 0:
                if cf_empty():
                    tactic_state = 250
                    flag_skip_front_1 = True
                else:
                    deploy_ff(1)          
                    tactic_state = 200
                
        # Hvata 2
        case 200:
            if get_side() == 1:
                move_on_angle(dist=0.3, dir=1, phi=math.pi)
            else:
                move_on_angle(dist=0.24, dir=1, phi=math.pi)
            tactic_state = 205
        case 205:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 210
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 200
        case 210:
            state, position = lift(1, -1)
            if state < 0:
                tactic_state = 215
        case 215:
            if lift_carry(1) < 0:
                tactic_state = 220
        # Stackuje se u zid
        case 220:
            move_on_angle(dist=0.18, dir=1, phi=math.pi, v_max=0.2)
            tactic_state = 225
        case 225:
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
                    if abs(get_GT().x_base) > 1.2875 - offset_d_tol and abs(get_GT().x_base) < 1.2875 + offset_d_tol:
                        offset_x = 1.2875
                    else:
                        offset_x = get_GT().x_base
                    get_GT().update_pose(
                        sign(get_GT().x_base) * abs(offset_x), 0.0, offset_phi, 101
                    )
                    tactic_state = 226
                else:
                    tactic_state = 230
            elif move_success() or move_interrupted() or move_failed():
                tactic_state = 230
        case 226:
            if get_update_pose_result() == -1:
                tactic_state = 230
        case 230:
            if not is_mech_running():
                mechanism_thread(1)
                tactic_state = 235
        case 235:
            move_on_direction(dist=0.2, dir=-1)
            tactic_state = 240
        case 240:
            if move_success() or move_stacked() or move_failed():
                if flag_skip_back:
                    flag_skip_back = False
                    tactic_state = 280
                else:
                    tactic_state = 250
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 235
        # Ostavlja u 3
        case 250:
            move_to_xy(x=-1.02, y=-0.68, dir=-1)
            tactic_state = 255
        case 255:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 260
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 250
        case 260:
            rotate_to_phi(phi=3 / 4 * math.pi)
            tactic_state = 265
        case 265:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 266
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 260
        case 266:
            if mechanism_drop(-1) < 0:
                tactic_state = 267
        case 267:
            move_on_direction(dist=0.1, dir=1)
            tactic_state = 268
        case 268:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                deploy_ff(-1)
                tactic_state = 269
        case 269:
            move_on_direction(dist=0.2, dir=-1)
            tactic_state = 270
        case 270:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 272
        case 272:
            move_on_direction(dist=0.2, dir=1)
            tactic_state = 275
        case 275:
            if move_success()or move_stacked() or move_failed():
                tactic_state = 278
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 272
        case 278:
            if mechanism_reset(-1) < 0:
                if flag_skip_front_1 == True:
                    tactic_state = 310
                else:
                    tactic_state = 280
        # Ostavlja u 1
        case 280:
            if get_side() == 1:
                move_to_xy(x=-1.20, y=-0.22, dir=1)
            else:
                move_to_xy(x=-1.18, y=-0.22, dir=1)
            tactic_state = 285
        case 285:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 286
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 280
        case 286:
            rotate_to_phi(phi=math.pi)
            tactic_state = 288
        case 288:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 290
        case 290:
            if mechanism_drop(1) < 0:
                tactic_state = 292
        case 292:
            move_on_direction(dist=0.2, dir=-1)
            tactic_state = 295
        case 295:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 298
        case 298:
            if mechanism_reset(1) < 0:
                tactic_state = 310
        # Prilazna za 1
        case 310:
            if get_side() == 1:
                move_to_xy(x=-1.0, y=0.185, dir=1)
            else:
                move_to_xy(x=-1.0, y=0.175, dir=1)
            tactic_state = 315
        case 315:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 320
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 380
                flag_skip_front_2 = True
        case 320:
            rotate_to_phi(phi=math.pi)
            tactic_state = 325
        case 325:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 330
        case 330:
            if snapshot_fsm(1, 10) < 0:
                if cf_empty():
                    tactic_state = 380
                    flag_skip_front_2 = True
                else:
                    deploy_ff(1)          
                    tactic_state = 340
        # Hvata 1
        case 340:
            # move_on_angle(dist=0.1525, dir=1, phi=math.pi)
            if get_side() == 1:
                move_to_xy(x=-1.1975, y=get_GT().y_base, dir=1)
            else:
                move_to_xy(x=-1.1775, y=get_GT().y_base, dir=1)
            tactic_state = 341
        case 341:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 342
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 340
        case 342:
            state, position = lift(1, -1)
            if state < 0:
                tactic_state = 343
        case 343:
            if lift_carry(1) < 0:
                tactic_state = 344
        # Stackuje se u zid
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
                    if abs(get_GT().x_base) > 1.2875 - offset_d_tol and abs(get_GT().x_base) < 1.2875 + offset_d_tol:
                        offset_x = 1.2875
                    else:
                        offset_x = get_GT().x_base
                    get_GT().update_pose(
                        sign(get_GT().x_base) * abs(offset_x), 0.0, offset_phi, 101
                    )
                    tactic_state = 350
                else:
                    tactic_state = 370
            elif move_success() or move_failed() or move_interrupted():
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
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 380
        case 380:
            if get_side() == 1:
                # move_to_xy(-1.0, -0.41, -1)
                move_to_xy(-1.0, -0.46, -1)
            else:
                move_to_xy(-1.0, -0.50, -1)
            tactic_state = 385
        case 385:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 390
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 380
        # Prilazna za 8
        case 390:
            if get_side() == 1:
                move_to_xy(x=0.9, y=-0.645, dir=1)
            else:
                move_to_xy(x=0.905, y=-0.52, dir=1)
            tactic_state = 395
        case 395:
            if move_success()or move_stacked() or move_failed() :
                tactic_state = 400
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 390
        case 400:
            rotate_to_phi(phi=math.pi)
            tactic_state = 410
        case 410:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 415
        case 415:
            if snapshot_fsm(-1, 10) < 0:
                if cb_empty():
                    tactic_state = 500
                    # flag_skip_back = True
                else:
                    deploy_ff(-1)          
                    tactic_state = 420
        # Hvata 8
        case 420:
            if get_side() == 1:
                move_on_angle(dist=0.27, dir=-1, phi=math.pi)
            else:
                move_on_angle(dist=0.29, dir=-1, phi=math.pi)
            tactic_state = 425
        case 425:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 430
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 420
        case 430:
            state, position = lift(-1, -1)
            if state < 0:
                tactic_state = 435
        case 435:
            if lift_carry(-1) < 0:
                tactic_state = 440
        # Stackuje se u zid
        case 440:
            move_on_angle(dist=0.15, dir=-1, phi=math.pi, v_max=0.2)
            tactic_state = 445
        case 445:
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
                    if abs(get_GT().x_base) > 1.2875 - offset_d_tol and abs(get_GT().x_base) < 1.2875 + offset_d_tol:
                        offset_x = 1.2875
                    else:
                        offset_x = get_GT().x_base
                    get_GT().update_pose(
                        sign(get_GT().x_base) * abs(offset_x), 0.0, offset_phi, 101
                    )
                    tactic_state = 446
                else:
                    tactic_state = 450
            elif move_success() or move_failed() or move_interrupted():
                tactic_state = 450
        case 446:
            if get_update_pose_result() == -1:
                tactic_state = 450
        case 447:
            state, position = lift(-1, -1)
            if state < 0:
                tactic_state = 448
        case 448:
            if lift_carry(-1) < 0:
                tactic_state = 450
        case 450:
            move_on_direction(dist=0.2, dir=1)
            tactic_state = 460
        case 460:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                if flag_skip_front_2 == True:
                    tactic_state = 3000
                else:
                    tactic_state = 500
                lift_to_rotating_ff(-1)
        # Ostavlja u 6
        case 500:
            if get_side() == 1: # plava
                move_to_xy(x=0.0, y=-0.55, dir=1)
            else: # zuta
                move_to_xy(x=0.05, y=-0.55, dir=1)
            tactic_state = 505
        case 505:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 510
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 500
        case 510:
            rotate_to_phi(phi=-0.5 * math.pi)
            tactic_state = 515
        case 515:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 530
        case 530:
            if mechanism_drop(1) < 0:
                tactic_state = 540
        case 540:
            move_on_direction(dist=0.2, dir=-1)
            tactic_state = 545
        case 545:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 548
        case 548:
            if mechanism_reset(1) < 0:
                tactic_state = 550
        case 550:
            move_on_direction(dist=0.30, dir=1)
            tactic_state = 555
        case 555:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 560
        case 560:
            move_on_direction(dist=0.5, dir=-1, v_max=0.75)
            tactic_state = 565
        case 565:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                # if flag_back_skipped:
                tactic_state = 570
                # else:
                #     tactic_state = 9800


        # Prilazna za 6
        case 570:
            if get_side() == 1: # plava
                move_to_xy(x=0.37, y=-0.48, dir=-1)
            else: # zuta
                move_to_xy(x=0.41, y=-0.48, dir=-1)     
            tactic_state = 575
        case 575:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 580
            # TODO: odradi ovo: cancel i idi u 580
            # elif get_GT().time > 
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 570
        case 580:
            rotate_to_phi(phi=-math.pi * 0.5)
            tactic_state = 585
        case 585:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 586
        case 586:
            if snapshot_fsm(1, 10) < 0:
                if cf_empty():
                    tactic_state = 800
                else:
                    deploy_ff(1)          
                    tactic_state = 590 
        case 590:
            if get_side() == 1:
                move_to_xy_unsided(x=get_GT().x_base, y=-0.75, dir=1)
            else:
                move_to_xy_unsided(x=get_GT().x_base, y=-0.678, dir=1)
            tactic_state = 600
        case 600:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 610
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 595
        case 610:
            state, position = lift(1, -1)
            if state < 0:
                tactic_state = 615
        case 615:
            if lift_carry(1) < 0:
                tactic_state = 900
        # Stackuje se u zid
        case 620:
            move_to_xy_unsided(x=get_GT().x_base, y=-0.8275, dir=1, v_max=0.2)
            tactic_state = 625
        case 625:
            if move_stacked():
                offset_phi = -math.pi * 0.5
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
                    # get_GT().update_pose(0.0, offset_y, offset_phi, 11)
                    # tactic_state = 626
                    tactic_state = 900
                else:
                    tactic_state = 900
            elif move_success() or move_failed() or move_interrupted():
                tactic_state = 900
        case 626:
            if get_update_pose_result() == -1:
                tactic_state = 900


        case 800:
            move_to_xy(x=0.35, y=-0.48, dir=1)   
            tactic_state = 805
        case 805:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 810
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 800
        case 810:
            rotate_to_phi(phi=math.pi * 0.5)
            tactic_state = 815
        case 815:
            if snapshot_fsm(1, 10) < 0:
                deploy_ff(1)
                tactic_state = 820
        case 820:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                if cf_empty():
                    tactic_state = 9800
                else:
                    deploy_ff(1)          
                    tactic_state = 825
        case 825:
            move_to_xy_unsided(x=get_GT().x_base, y=-0.2, dir=1)
            tactic_state = 830
        case 830:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 835
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 825
        case 835:
            state, position = lift(1, -1)
            if state < 0:
                tactic_state = 840
        case 840:
            if lift_carry(1) < 0:
                tactic_state = 850
        case 850:
            move_on_direction(dist=0.15, dir=-1)
            tactic_state = 855
        case 855:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 900
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 850


        case 900:
            if not is_mech_running():
                mechanism_thread(1)
                tactic_state = 910
        case 910:
            move_on_direction(dist=0.15, dir=-1)
            tactic_state = 920
        case 920:
            if move_success() or move_stacked() or move_failed():
                    tactic_state = 1000
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 910
        # Ostavlja u 5
        case 1000:
            move_to_xy(x=-0.125, y=-0.35, dir=-1)
            tactic_state = 1010
        case 1010:
            if move_success() or move_stacked() or move_failed():
                    tactic_state = 1015
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 1000
        case 1015:
            rotate_to_phi(phi=0.33*math.pi)
            tactic_state = 1020
        case 1020:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 1025
        case 1025:
            if mechanism_drop(1) < 0:
                tactic_state = 1030
        case 1030:
            move_to_xy(x=-0.25, y=-0.45, dir= -1)
            tactic_state = 1040
        case 1040:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 9800
                if get_GT().time >= 94.0:
                    tactic_state = 10000
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 1030
                if get_GT().time >= 94.0:
                    tactic_state = 10000
        
        # Prilazna za 7
        case 3000:
            if get_side() == 1:
                move_to_xy(x=1.0, y=0.12, dir=1)
            else:
                move_to_xy(x=1.0, y=0.23, dir=1)
            tactic_state = 3015
        case 3015:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 3020
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 3010
        case 3020:
            rotate_to_phi(phi=0.0)
            tactic_state = 3025
        case 3025:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 3030
        case 3030:
            if snapshot_fsm(1, 10) < 0:
                if cf_empty():
                    tactic_state = 3072
                    flag_skip_front_2 = True
                else:
                    deploy_ff(1)          
                    tactic_state = 3040
                    flag_skip_front_2 = False
        # Hvata 7
        case 3040:
            if get_side() == 1:
                move_to_xy(x=1.1975, y=get_GT().y_base, dir=1)
            else:
                move_to_xy(x=1.1775, y=get_GT().y_base, dir=1)
            tactic_state = 3041
        case 3041:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 3042
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 3040
        case 3042:
            state, position = lift(1, -1)
            if state < 0:
                tactic_state = 3043
        case 3043:
            if lift_carry(1) < 0:
                # if get_GT().time >= 89.0:
                #     tactic_state = 9800
                # else:
                    tactic_state = 3044
        # Stackuje se u zid
        case 3044:
            move_on_angle(dist=0.16, dir=1, phi=0.0, v_max=0.25)
            tactic_state = 3046
        case 3046:
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
                    if abs(get_GT().x_base) > 1.2875 - offset_d_tol and abs(get_GT().x_base) < 1.2875 + offset_d_tol:
                        offset_x = 1.2875
                    else:
                        offset_x = get_GT().x_base
                    get_GT().update_pose(
                        sign(get_GT().x_base) * abs(offset_x), 0.0, offset_phi, 101
                    )
                    tactic_state = 3050
                else:
                    tactic_state = 3070
            elif move_success() or move_failed() or move_interrupted():
                tactic_state = 3050
        case 3050:
            if get_update_pose_result() == -1:
                tactic_state = 3070
        case 3070:
            if not is_mech_running():
                mechanism_thread(1)
                # if get_GT().time >= 90.0:
                #     tactic_state = 9800
                # else:
                tactic_state = 3072
        case 3072:
            move_on_direction(dist=0.2, dir=-1)
            tactic_state = 3075
        case 3075:
            if move_success() or move_stacked() or move_failed() or move_interrupted():
                tactic_state = 3080
        case 3080:
            if get_side() == 1:
                move_to_xy(x=1.0, y=-0.2, dir=1)
            else:
                move_to_xy(x=1.0, y=-0.2, dir=1)
            tactic_state = 3081
        case 3081:
            if move_success() or move_stacked() or move_failed():
                tactic_state = 500
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 3080
        
        case 9800:
            move_to_xy(0.0, -0.45, 1)
            tactic_state = 9900
        case 9900:
            if move_success() or move_failed()or move_stacked():
                undeploy_ff(-1)
                tactic_state = 9920
                print("Current time: " + str(get_GT().time))
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 9800
        case 9920:
            rotate_to_phi(phi=0.0)
            tactic_state = 9925
        case 9925:
            if move_success() or move_failed()or move_stacked() or move_interrupted():
                tactic_state = 9950
        case 9950:
            if get_GT().time >= 93.0:
                tactic_state = 10000

        case 10000:
            if get_GT().time < 95.0:
                move_to_xy(-1.0, -0.6, -1)
                tactic_state = 10100
            else:
                tactic_state = -1
        case 10100:
            if move_success()or move_failed()or move_stacked():
                tactic_state = 10200
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 10000
        case 10200:
            if get_side() == 1:
                move_to_xy(-1.15, 0.76, 1)
            else:
                move_to_xy(-1.15, 0.74, 1)
            tactic_state = 10300
        case 10300:
            if move_success()or move_failed()or move_stacked():
                undeploy_ff(1)
                tactic_state = -1
            elif move_interrupted():
                time.sleep(0.1)
                tactic_state = 10200
                
        case -1:
            print("Tactic 0 finished.")
    return tactic_state
