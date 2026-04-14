from ros381_tactics.get_set import *

# pylint: disable=import-error
import ros381_tactics_py  # type: ignore

# pylint: enable=import-error
import math
import time
import threading

init_state = 0
lift_front_id = lift_back_id = 69
clan1_front_id = clan2_front_id = clan3_front_id = clan4_front_id = 69
clan1_back_id = clan2_back_id = clan3_back_id = clan4_back_id = 69
cursor_id = 69
lift_up_pos = lift_down_pos = lift_carry_pos = lift_rotating_pos = lift_dropoff_pos = 69
cursor_up_pos = 69
clanL_up_pos = clanL_down_pos = clanR_up_pos = clanR_down_pos = clanL_undep_pos = (
    clanR_undep_pos
) = 69


mech_state = 0
mlift_id = lift_front_id
mcl1_id = mcl2_id = mcl3_id = mcl4_id = 0
mcl1_pos = mcl2_pos = mcl3_pos = mcl4_pos = 0
mvf = mvb = False


mechanism_thread_obj = None


def is_mech_running():
    return mechanism_thread_obj is not None and mechanism_thread_obj.is_alive()


def mechanism_thread(side):
    global mechanism_thread_obj, mech_state

    def worker():
        global mech_state

        mech_state = 0

        while True:
            state = mechanism_prep(side)

            if state < 0:
                break

            time.sleep(0.1)

    mechanism_thread_obj = threading.Thread(target=worker, daemon=True)
    mechanism_thread_obj.start()


# TODO: fire and forget u thread
def mechanism_prep(side):
    global mech_state, mlift_id
    global mcl1_id, mcl2_id, mcl3_id, mcl4_id
    global mcl1_pos, mcl2_pos, mcl3_pos, mcl4_pos
    global mvf, mvb
    color = sided_color()

    match mech_state:
        case 0:
            # 0. na osnovu strane inicijalizuj: mlift_id, mcl_ids, mcl_positions, mvf, mvb
            if side == 1:
                mlift_id = lift_front_id
                mcl1_id = clan1_front_id
                mcl2_id = clan2_front_id
                mcl3_id = clan3_front_id
                mcl4_id = clan4_front_id
                cf_local = get_cf()
                print("cf_local: ", cf_local)
                mcl1_pos = clanL_up_pos if cf_local[0] == color else clanL_down_pos
                mcl2_pos = clanL_up_pos if cf_local[1] == color else clanL_down_pos
                mcl3_pos = clanR_up_pos if cf_local[2] == color else clanR_down_pos
                mcl4_pos = clanR_up_pos if cf_local[3] == color else clanR_down_pos
                clear_cf()
                mvf = True
                mvb = False
            else:
                mlift_id = lift_back_id
                mcl1_id = clan1_back_id
                mcl2_id = clan2_back_id
                mcl3_id = clan3_back_id
                mcl4_id = clan4_back_id
                cb_local = get_cb()
                print("cb_local: ", cb_local)
                mcl1_pos = clanL_up_pos if cb_local[0] == color else clanL_down_pos
                mcl2_pos = clanL_up_pos if cb_local[1] == color else clanL_down_pos
                mcl3_pos = clanR_up_pos if cb_local[2] == color else clanR_down_pos
                mcl4_pos = clanR_up_pos if cb_local[3] == color else clanR_down_pos
                clear_cb()
                mvf = False
                mvb = True
            mech_state = 10
        case 10:
            # 1. lift na rotating
            ax_move(mlift_id, lift_rotating_pos, 1000, 20)
            mech_state = 15
        case 15:
            if get_ax_move_result() < 0:
                mech_state = 20
        case 20:
            # 2. bulk move za clanove
            ax_bulk_move(
                [
                    (mcl1_id, mcl1_pos, 1000, 200),
                    (mcl2_id, mcl2_pos, 1000, 200),
                    (mcl3_id, mcl3_pos, 1000, 200),
                    (mcl4_id, mcl4_pos, 1000, 200),
                ]
            )
            mech_state = 25
        case 25:
            if get_ax_bulk_move_result() < 0:
                mech_state = 30
        case 30:
            # 3. lift na dropoff
            ax_move(mlift_id, lift_dropoff_pos, 1000, 1000)
            mech_state = -1
            
    return mech_state


mechd_state = 0
mdlift_id = lift_front_id
mdcl1_id = mdcl2_id = mdcl3_id = mdcl4_id = 0
mdcl1_pos = mdcl2_pos = mdcl3_pos = mdcl4_pos = 0
mdvf = mdvb = False


def mechanism_drop(side):
    global mechd_state, mdlift_id
    global mdcl1_id, mdcl2_id, mdcl3_id, mdcl4_id
    global mdcl1_pos, mdcl2_pos, mdcl3_pos, mdcl4_pos
    global mdvf, mdvb

    match mechd_state:
        case 0:
            if side == 1:
                mdlift_id = lift_front_id
                mdcl1_id = clan1_front_id
                mdcl2_id = clan2_front_id
                mdcl3_id = clan3_front_id
                mdcl4_id = clan4_front_id
                mdvf = True
                mdvb = False
            else:
                mdlift_id = lift_back_id
                mdcl1_id = clan1_back_id
                mdcl2_id = clan2_back_id
                mdcl3_id = clan3_back_id
                mdcl4_id = clan4_back_id
                mdvf = False
                mdvb = True
            mechd_state = 40
        # case 30:
        #     # 3. lift na dropoff
        #     ax_move(mdlift_id, lift_dropoff_pos, 1000, 400)
        #     mechd_state = 35
        # case 35:
        #     if get_ax_move_result() < 0:
        #         mechd_state = 40
        case 40:
            # 4. iskljuci vakuum
            get_GT().remove_vacuum(mdvf, mdvb)
            mechd_state = 50
        case 50:
            # 3. lift na dropoff
            ax_move(mdlift_id, lift_dropoff_pos + 72, 70, 56)
            mechd_state = 55
        case 55:
            if get_ax_move_result() < 0:
                mechd_state = -1
        case -1:
            mechd_state = 0

    return mechd_state


mech_reset_state = 0
mrcl1_id = mrcl2_id = mrcl3_id = mrcl4_id = 1
mrlift_id = lift_front_id


def mechanism_reset(side):
    global mech_reset_state, mrlift_id
    global mrcl1_id, mrcl2_id, mrcl3_id, mrcl4_id
    match mech_reset_state:
        case 0:
            if side == 1:
                mrlift_id = lift_front_id
                mrcl1_id = clan1_front_id
                mrcl2_id = clan2_front_id
                mrcl3_id = clan3_front_id
                mrcl4_id = clan4_front_id
            else:
                mrlift_id = lift_back_id
                mrcl1_id = clan1_back_id
                mrcl2_id = clan2_back_id
                mrcl3_id = clan3_back_id
                mrcl4_id = clan4_back_id
            mech_reset_state = 5
        case 5:
            # 5. lift na rotating
            ax_move(mrlift_id, lift_rotating_pos, 1000, 500)
            mech_reset_state = 10
        case 10:
            if get_ax_move_result() < 0:
                mech_reset_state = 15
        case 15:
            # 6. bulk move za clanove
            ax_bulk_move(
                [
                    (mrcl1_id, clanL_down_pos, 1000, 500),
                    (mrcl2_id, clanL_down_pos, 1000, 500),
                    (mrcl3_id, clanR_down_pos, 1000, 500),
                    (mrcl4_id, clanR_down_pos, 1000, 500),
                ]
            )
            # mech_reset_state = 20
            mech_reset_state = -1
        case 20:
            if get_ax_bulk_move_result() < 0:
                mech_reset_state = -1
        case -1:
            mech_reset_state = 0

    return mech_reset_state


cursor_state = 0


def cursor(position):
    global cursor_state, cursor_id
    match cursor_state:
        case 0:
            if position == 1:
                cursor_state = 10
            else:
                cursor_state = 20
        case 10:
            ax_move(cursor_id, cursor_up_pos, 1000, 150)
            cursor_state = 11
        case 11:
            if get_ax_move_result() < 0:
                cursor_state = -1
        case 20:
            ax_hybrid_move(cursor_id, 1000, 0.2, 200)
            cursor_state = 21
        case 21:
            if get_ax_hybrid_move_result() < 0:
                cursor_state = 22
        case 22:
            pos = get_ax_hybrid_end_position()
            print("Cursor reached position " + str(pos))
            cursor_state = -1
        case -1:
            cursor_state = 0
    return cursor_state


def deploy_ff(side):
    print("Deploying: " + str(side))
    if side == 1:
        ax_bulk_move(
            [
                (clan1_front_id, clanL_down_pos, 1000, 200),
                (clan2_front_id, clanL_down_pos, 1000, 200),
                (clan3_front_id, clanR_down_pos, 1000, 200),
                (clan4_front_id, clanR_down_pos, 1000, 200),
            ]
        )
    else:
        ax_bulk_move(
            [
                (clan1_back_id, clanL_down_pos, 1000, 200),
                (clan2_back_id, clanL_down_pos, 1000, 200),
                (clan3_back_id, clanR_down_pos, 1000, 200),
                (clan4_back_id, clanR_down_pos, 1000, 200),
            ]
        )


def undeploy_ff(side):
    print("Undeploying: " + str(side))
    if side == 1:
        ax_bulk_move(
            [
                (clan1_front_id, clanL_undep_pos, 1000, 200),
                (clan2_front_id, clanL_undep_pos, 1000, 200),
                (clan3_front_id, clanR_undep_pos, 1000, 200),
                (clan4_front_id, clanR_undep_pos, 1000, 200),
            ]
        )
    else:
        ax_bulk_move(
            [
                (clan1_back_id, clanL_undep_pos, 1000, 200),
                (clan2_back_id, clanL_undep_pos, 1000, 200),
                (clan3_back_id, clanR_undep_pos, 1000, 200),
                (clan4_back_id, clanR_undep_pos, 1000, 200),
            ]
        )


lift_state = 0
lift_pos = 511
lift_dpos = 200
lift_retpos = -1
vf = False
vb = False
lcl1_id = lcl2_id = lcl3_id = lcl4_id = 1


def lift(side, state):
    global lift_state, lift_id, lift_pos, lift_dpos, lift_retpos, vf, vb, lcl1_id, lcl2_id, lcl3_id, lcl4_id
    match lift_state:
        case 0:
            lift_retpos = -1
            if side == 1:
                lift_id = lift_front_id
                vf = True
                vb = False
                lcl1_id = clan1_front_id
                lcl2_id = clan2_front_id
                lcl3_id = clan3_front_id
                lcl4_id = clan4_front_id
            else:
                lift_id = lift_back_id
                vf = False
                vb = True
                lcl1_id = clan1_back_id
                lcl2_id = clan2_back_id
                lcl3_id = clan3_back_id
                lcl4_id = clan4_back_id
            if state == 1:
                lift_pos = lift_up_pos
                lift_dpos = -200
                vf = False
                vb = False
            else:
                lift_pos = lift_down_pos
                lift_dpos = 200
            lift_state = 5
        case 5:
            ax_bulk_move(
                [
                    (lcl1_id, clanL_down_pos, 1000, 200),
                    (lcl2_id, clanL_down_pos, 1000, 200),
                    (lcl3_id, clanR_down_pos, 1000, 200),
                    (lcl4_id, clanR_down_pos, 1000, 200),
                ]
            )
            lift_state = 8
        case 8:
            if get_ax_bulk_move_result() < 0:
                lift_state = 10
        case 10:
            ax_move(lift_id, lift_pos, 1000, 200)
            lift_state = 20
        case 20:
            if get_ax_move_result() < 0:
                lift_state = 30
        case 30:
            get_GT().add_vacuum(vf, vb)
            lift_state = 40
        case 40:
            ax_hybrid_move(lift_id, 1000, 0.2, lift_dpos)
            lift_state = 50
        case 50:
            if get_ax_hybrid_move_result() < 0:
                lift_state = 60
        case 60:
            lift_retpos = get_ax_hybrid_end_position()
            lift_state = -1
        case -1:
            lift_state = 0
    return lift_state, lift_retpos


def lift_carry(side):
    global lift_id, lift_state#, lift_dpos
    match lift_state:
        case 0:
            # lift_dpos = 200
            if side == 1:
                lift_id = lift_front_id
            else:
                lift_id = lift_back_id
            lift_state = 10
        # case 2:
        #     ax_hybrid_move(lift_id, 1000, 0.2, lift_dpos)
        #     lift_state = 5
        # case 5:
        #     if get_ax_hybrid_move_result() < 0:
        #         lift_state = 10
        case 10:
            ax_move(lift_id, lift_carry_pos, 1000, 100)
            lift_state = 20
        case 20:
            if get_ax_move_result() < 0:
                lift_state = -1
        case -1:
            lift_state = 0
    return lift_state


def lift_to_rotating_ff(side):
    global llift_rotating_pos
    if side == 1:
        lift_id = lift_front_id
        ax_move(lift_front_id, lift_rotating_pos, 1000, 100)
    else:
        lift_id = lift_back_id
        ax_move(lift_id, lift_back_id, 1000, 100)


def get_ax_move_result():
    return get_GT().ax_move_result_


def get_ax_bulk_move_result():
    return get_GT().ax_bulk_move_result_


def get_ax_hybrid_move_result():
    return get_GT().ax_hybrid_move_result_


def get_ax_hybrid_end_position():
    return get_GT().ax_hybrid_end_position_


def ax_move(id, position, velocity, position_tolerance):
    GT = get_GT()
    goal = ros381_tactics_py.AxMoveGoal()
    goal.id = id
    goal.position = position
    goal.velocity = velocity
    goal.position_tolerance = position_tolerance
    GT.ax_move_goal(goal)


def ax_bulk_move(moves):
    GT = get_GT()
    goals = []
    for move in moves:
        goal = ros381_tactics_py.AxMoveGoal()
        goal.id = move[0]
        goal.position = move[1]
        goal.velocity = move[2]
        goal.position_tolerance = move[3]
        goals.append(goal)

    GT.ax_bulk_move_goal(goals)


def ax_hybrid_move(id, velocity, zero_time, delta_pos):
    get_GT().ax_hybrid_move_goal(id, velocity, zero_time, delta_pos)


reset_undeployed_state = 0


def reset_to_deployed(side):
    global reset_undeployed_state

    match reset_undeployed_state:
        case 0:
            if side == 1:
                ax_move(lift_front_id, lift_rotating_pos, 1000, 500)
            else:
                ax_move(lift_back_id, lift_rotating_pos, 1000, 500)
            reset_undeployed_state = 10
        case 10:
            if get_ax_move_result() < 0:
                reset_undeployed_state = 20
        case 20:
            if side == 1:
                ax_bulk_move(
                    [
                        (clan1_front_id, clanL_down_pos, 1000, 500),
                        (clan2_front_id, clanL_down_pos, 1000, 500),
                        (clan3_front_id, clanR_down_pos, 1000, 500),
                        (clan4_front_id, clanR_down_pos, 1000, 500),
                    ]
                )
            else:
                ax_bulk_move(
                    [
                        (clan1_back_id, clanL_down_pos, 1000, 500),
                        (clan2_back_id, clanL_down_pos, 1000, 500),
                        (clan3_back_id, clanR_down_pos, 1000, 500),
                        (clan4_back_id, clanR_down_pos, 1000, 500),
                    ]
                )
            reset_undeployed_state = 30
        case 30:
            if get_ax_bulk_move_result() < 0:
                reset_undeployed_state = -1
        case -1:
            reset_undeployed_state = 0

    return reset_undeployed_state


def init_ax():
    global init_state
    match init_state:
        case 0:
            ax_hybrid_move(lift_front_id, 1000, 0.2, -200)
            init_state = 10
        case 10:
            if get_ax_hybrid_move_result() < 0:
                init_state = 40
        case 40:
            ax_hybrid_move(lift_back_id, 1000, 0.2, -200)
            init_state = 50
        case 50:
            if get_ax_hybrid_move_result() < 0:
                init_state = 60
        case 60:
            ax_bulk_move(
                [
                    (lift_front_id, lift_rotating_pos, 1000, 500),
                    (lift_back_id, lift_rotating_pos, 1000, 500),
                ]
            )
            init_state = 76
        case 76:
            if get_ax_bulk_move_result() < 0:
                init_state = 80

        case 80:
            ax_bulk_move(
                [
                    (clan1_front_id, clanL_down_pos, 1000, 500),
                    (clan2_front_id, clanL_down_pos, 1000, 500),
                    (clan3_front_id, clanR_down_pos, 1000, 500),
                    (clan4_front_id, clanR_down_pos, 1000, 500),
                    (clan1_back_id, clanL_down_pos, 1000, 500),
                    (clan2_back_id, clanL_down_pos, 1000, 500),
                    (clan3_back_id, clanR_down_pos, 1000, 500),
                    (clan4_back_id, clanR_down_pos, 1000, 500),
                ]
            )
            init_state = 90
        case 90:
            if get_ax_bulk_move_result() < 0:
                init_state = 100

        case 100:
            ax_bulk_move(
                [
                    (clan1_front_id, clanL_undep_pos, 1000, 500),
                    (clan2_front_id, clanL_undep_pos, 1000, 500),
                    (clan3_front_id, clanR_undep_pos, 1000, 500),
                    (clan4_front_id, clanR_undep_pos, 1000, 500),
                    (clan1_back_id, clanL_undep_pos, 1000, 500),
                    (clan2_back_id, clanL_undep_pos, 1000, 500),
                    (clan3_back_id, clanR_undep_pos, 1000, 500),
                    (clan4_back_id, clanR_undep_pos, 1000, 500),
                ]
            )
            init_state = 110
        case 110:
            if get_ax_bulk_move_result() < 0:
                init_state = 180
        case 180:
            ax_move(cursor_id, cursor_up_pos, 1000, 500)
            init_state = 190
        case 190:
            if get_ax_move_result() < 0:
                init_state = 99
        case 99:
            if get_ax_bulk_move_result() < 0:
                init_state = -1
        case -1:
            return True
    return False


def load_ax_params():
    global lift_front_id, lift_back_id
    global clan1_front_id, clan2_front_id, clan3_front_id, clan4_front_id
    global clan1_back_id, clan2_back_id, clan3_back_id, clan4_back_id
    global cursor_id
    global lift_up_pos, lift_down_pos, lift_carry_pos, lift_rotating_pos, lift_dropoff_pos
    global cursor_up_pos
    global clanL_up_pos, clanL_down_pos, clanR_up_pos, clanR_down_pos, clanL_undep_pos, clanR_undep_pos

    GT = get_GT()

    # IDs
    lift_front_id = GT.lift_front_id_
    lift_back_id = GT.lift_back_id_
    clan1_front_id = GT.clan1_front_id_
    clan2_front_id = GT.clan2_front_id_
    clan3_front_id = GT.clan3_front_id_
    clan4_front_id = GT.clan4_front_id_
    clan1_back_id = GT.clan1_back_id_
    clan2_back_id = GT.clan2_back_id_
    clan3_back_id = GT.clan3_back_id_
    clan4_back_id = GT.clan4_back_id_
    cursor_id = GT.cursor_id_

    # Positions
    lift_up_pos = GT.lift_up_pos_
    lift_down_pos = GT.lift_down_pos_
    lift_carry_pos = GT.lift_carry_pos_
    lift_rotating_pos = GT.lift_rotating_pos_
    lift_dropoff_pos = GT.lift_dropoff_pos_
    cursor_up_pos = GT.cursor_up_pos_
    clanL_up_pos = GT.clanL_up_pos_
    clanL_down_pos = GT.clanL_down_pos_
    clanR_up_pos = GT.clanR_up_pos_
    clanR_down_pos = GT.clanR_down_pos_
    clanL_undep_pos = GT.clanL_undep_pos_
    clanR_undep_pos = GT.clanR_undep_pos_

    # Print AX parameters
    print(f"Lift IDs: front: {lift_front_id}, back: {lift_back_id}")
    print(
        f"Clan Front IDs: {clan1_front_id}, {clan2_front_id}, {clan3_front_id}, {clan4_front_id}"
    )
    print(
        f"Clan Back IDs: {clan1_back_id}, {clan2_back_id}, {clan3_back_id}, {clan4_back_id}"
    )
    print(f"Cursor ID: {cursor_id}")
    print(
        f"Lift Positions: up: {lift_up_pos}, down: {lift_down_pos}, carry: {lift_carry_pos}, rotating: {lift_rotating_pos}, ddropoff: {lift_dropoff_pos}"
    )
    print(f"Cursor Position: up: {cursor_up_pos}")
    print(
        f"ClanL Positions: up: {clanL_up_pos}, down: {clanL_down_pos}, undeployed: {clanL_undep_pos}"
    )
    print(
        f"ClanR Positions: up: {clanR_up_pos}, down: {clanR_down_pos}, undeployed: {clanR_undep_pos}"
    )
