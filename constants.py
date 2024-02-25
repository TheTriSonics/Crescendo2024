from misc import is_sim

class RobotMap():

    driver_controller = 0
    commander_controller_1 = 1
    commander_controller_2 = 2

    # CAN assignments
    front_right_drive = 11
    front_right_turn = 21
    front_right_turn_encoder = 31

    front_left_drive = 12
    front_left_turn = 22
    front_left_turn_encoder = 32

    back_right_drive = 13
    back_right_turn = 23
    back_right_turn_encoder = 33

    back_left_drive = 14
    back_left_turn = 24
    back_left_turn_encoder = 34

    if is_sim():
        intake_feed = 5
        intake_tilt = 6
        intake_tilt_encoder_a = 0
        intake_tilt_encoder_b = 1
    else:
        intake_feed = 51
        intake_tilt = 52
        intake_tilt_encoder_a = 0
        intake_tilt_encoder_b = 1

    intake_divert = 7

    amp_lift = 53
    amp_feed = 54

    shooter_left = 55
    shooter_right = 56
    shooter_feed = 57
    shooter_rotate_left = 58
    shooter_rotate_right = 59

    climber_left = 9990
    climber_right = 9989

    pigeon2_id = 41

    #Commander button assignments
    intake_ready = 0
    intake_eject = 1

    queue_note_shooter = 2
    queue_note_amp = 3
    queue_note_back_to_home = 4

    shooter_aim_otf = 5
    shooter_aim_safe = 6
    shooter_aim_sub = 7

    shooter_shoot = 8

    amp_lift_amp = 9
    amp_lift_trap = 10

    amp_eject = 11

    #Commander Override Buttons
    intake_tilt_up = 12
    intake_tilt_down = 13

    intake_roller_in = 14
    intake_roller_out = 15



