from misc import is_sim


class RobotMotorMap():
    # REAL CAN ASSIGNMENTS
    # Swerve Motors
    # front_left_drive = 12
    # front_right_drive = 11
    # back_left_drive = 14
    # back_right_drive = 13

    # front_left_turn = 22
    # front_right_turn = 21
    # back_left_turn = 24
    # back_right_turn = 23
    #
    # Cancoders
    # front_left_drive_encoder = 32
    # front_right_drive_encoder = 31
    # back_left_drive_encoder = 34
    # back_right_drive_encoder = 33

    # MIN CAN ID = 1 DO NOT USE ZERO FOR CAN ID
    # MAX CAN ID = 62
    # Can Ranges: Swerve Motors = 11-24, Swerve Cancoders = 31-34, Pigeon = 35, Shooter = 41-45, Intake = 51-52, Amp = 54-55, Climber = 57-58

    ### CHASSIS BOT ###
    # Swerve
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
    ### CHASSIS BOT ###

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

    # Amp Motors
    amp_lift_motor = 54
    amp_feed_motor = 55

    # Climber Motors
    climber_motor_left = 57
    climber_motor_right = 58

    # Shooter Motors
    shooter_motor_left = 41
    shooter_motor_right = 42
    shooter_motor_feed = 43
    shooter_motor_tilt_left = 44
    shooter_motor_tilt_right = 45


class RobotSensorMap():
    # Gyro
    pigeon2_id = 35

    # Limit Switches DIO
    amp_lift_bottom_limit_switch = 0  # May not be wired to DIO

    # Photoeyes DIO
    intake_front_photoeye = 0
    intake_hold_photoeye = 1
    amp_hold_photoeye = 2
    shooter_hold_photoeye = 3

    # Encoders DIO
    intake_tilt_encoder = 4
    shooter_tilt_encoder = 5
    climber_left_encoder = 6
    climber_right_encoder = 7


class RobotButtonMap():
    # Controller ids
    driver_controller = 0
    commander_controller_1 = 1
    commander_controller_2 = 2

    # Button Panel List
    # INTAKE: Intake Ready, Intake Eject
    # SHOOTER: Load Shooter, On The Fly(OTF) Aim, Safe Spot Aim, Against Sub Aim, Shoot
    # AMP: Load Amp, Go Amp Height, Go Trap Height, Eject Note
    # OVERRIDES: Intake Up, Intake Down, Intake Feed In, Intake Feed Out
    # MISC: Note Return to Home

    # Commander button assignments
    intake_ready = 0
    intake_eject = 1

    load_note_shooter = 2
    load_note_amp = 3
    load_note_back_to_home = 4

    shooter_aim_otf = 5
    shooter_aim_safe = 6
    shooter_aim_sub = 7
    shooter_shoot = 8

    amp_lift_amp = 9
    amp_lift_trap = 10
    amp_eject = 11

    # Commander Override Buttons
    intake_tilt_up = 12
    intake_tilt_down = 13

    intake_roller_in = 14
    intake_roller_out = 15
