from misc import is_sim


class RobotPIDConstants():
    note_tracking_pid = (0.04, 0, 0)


class RobotMotorMap():
    # REAL CAN ASSIGNMENTS
    # Swerve Motors
    # front_left_drive = 11
    # front_right_drive = 12
    # back_left_drive = 13
    # back_right_drive = 14

    # front_left_turn = 21
    # front_right_turn = 22
    # back_left_turn = 23
    # back_right_turn = 24
    #
    # Cancoders
    # front_left_drive_encoder = 31
    # front_right_drive_encoder = 32
    # back_left_drive_encoder = 33
    # back_right_drive_encoder = 34

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
        intake_motor_feed = 5
        intake_motor_tilt = 6
    else:
        intake_motor_feed = 51
        intake_motor_tilt = 52

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
    amp_lift_bottom_limit_switch = 0

    # Photoeyes DIO
    intake_front_photoeye = 1
    intake_hold_photoeye = 2
    amp_hold_photoeye = 3
    shooter_hold_photoeye = 4

    # Encoders DIO
    intake_tilt_encoder = 5
    shooter_tilt_encoder = 6
    climber_left_encoder = 7
    climber_right_encoder = 8

    # Addressable LEDS PWM
    addressable_leds = 0


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
    ### Controller 1 ###
    intake_ready = 0
    intake_eject = 1

    load_note_shooter = 2
    load_note_amp = 3
    load_note_back_to_home = 4

    shooter_aim_otf = 5
    shooter_aim_safe = 6
    shooter_aim_sub = 7
    shooter_shoot = 8

    ### Controller 2 ###
    amp_lift_amp = 0
    amp_lift_trap = 1
    amp_lift_home = 2
    amp_eject = 3

    # Commander Override Buttons
    intake_tilt_up = 4
    intake_tilt_down = 5

    intake_roller_in = 6
    intake_roller_out = 7
