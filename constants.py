from misc import is_sim


class RobotPIDConstants():
    note_tracking_pid = (0.07, 0, 0.004)
    note_translate_pid = note_tracking_pid
    speaker_tracking_pid = (0.06, 0, 0)
    # well, guess no, we don't want PID for that.
    # It makes it squishy
    straight_drive_pid = [0.04, 0, 0.001] # JRD Was 0.015, 0, 0.001


class RobotMotorMap():
    # REAL CAN ASSIGNMENTS

    # Swerve Motors
    front_left_drive = 11
    front_right_drive = 12
    back_left_drive = 13
    back_right_drive = 14

    front_left_turn = 21
    front_right_turn = 22
    back_left_turn = 23
    back_right_turn = 24

    # Cancoders
    front_left_turn_encoder = 31
    front_right_turn_encoder = 32
    back_left_turn_encoder = 33
    back_right_turn_encoder = 34

    # MIN CAN ID = 1 DO NOT USE ZERO FOR CAN ID
    # MAX CAN ID = 62
    # Can Ranges: Swerve Motors = 11-24, Swerve Cancoders = 31-34, Pigeon = 35,
    # Shooter = 41-45, Intake = 51-52, Amp = 54-55, Climber = 57-58

    # ## CHASSIS BOT ## #
    # Swerve
    # front_right_drive = 11
    # front_right_turn = 21
    # front_right_turn_encoder = 31

    # front_left_drive = 12
    # front_left_turn = 22
    # front_left_turn_encoder = 32

    # back_right_drive = 13
    # back_right_turn = 23
    # back_right_turn_encoder = 33

    # back_left_drive = 14
    # back_left_turn = 24
    # back_left_turn_encoder = 34
    # ## CHASSIS BOT ## #

    if is_sim():
        intake_motor_feed = 5
    else:
        intake_motor_feed = 51

    amp_lift_motor = 54
    amp_feed_motor = 55

    # Climber Motors
    climber_motor_left = 57
    climber_motor_right = 58

    # Shooter Motors
    shooter_motor_left = 41
    shooter_motor_right = 42

    shooter_motor_feed_right = 46
    shooter_motor_feed_left = 47

    shooter_motor_tilt_left = 44
    shooter_motor_tilt_right = 45


class RobotSensorMap():
    # Gyro
    pigeon2_id = 35

    # Limit Switches DIO
    amp_lift_bottom_limit_switch = 9

    # Photoeyes DIO
    intake_front_photoeye = 2
    intake_hold_photoeye = 1
    amp_hold_photoeye = 0
    shooter_hold_photoeye = 3

    # Encoders DIO
    shooter_tilt_encoder = 7
    climber_left_encoder = 4
    climber_right_encoder = 5

    # Addressable LEDS PWM
    addressable_leds = 0


class RobotButtonMap():
    # Controller ids
    driver_controller = 0
    commander_controller_1 = 1
    commander_controller_2 = 2

    # Driver button assignments
    note_tracking = 5  # Left bumper
    toggle_field_relative = 3
    speaker_tracking = 6  # Right bumper
    slow_drive_mode = 4  # Y Button
    flip_heading = 1
    swap_direction = 2


    RESET_YAW = 8

    # Button Panel List
    # INTAKE: Intake Ready, Intake Eject
    # SHOOTER: Load Shooter, On The Fly(OTF) Aim, Safe Spot Aim,
    #          Against Sub Aim, Shoot
    # AMP: Load Amp, Go Amp Height, Go Trap Height, Eject Note
    # OVERRIDES: Intake Up, Intake Down, Intake Feed In, Intake Feed Out
    # MISC: Note Return to Home

    # Commander button assignments
    # # Controller 1 ## #
    intake_ready_c1 = 9
    intake_eject_c1 = 8

    load_note_shooter_c1 = 7

    shooter_aim_sub_c1 = 6
    shooter_aim_otf_c1 = 3
    shooter_aim_safe_c1 = 2

    shooter_override_up_c1 = 4
    shooter_override_down_c1 = 5

    # ## Controller 2 ## #
    load_note_amp_c2 = 6

    amp_lift_home_c2 = 8
    amp_lift_amp_c2 = 11
    amp_lift_trap_c2 = 10

    # Unused
    amp_override_up_c2 = 9
    amp_override_down_c2 = 7

    amp_dump_note_c2 = 2

    shooter_shoot_c2 = 4

    shooter_spin_c2 = 3

    # Reset odometry to whatever vision thinks we're at.
    reset_odometry_c2 = 5
    override_shooter_speed_c2 = 1

    # Not used?
    climber_up_c2 = 5
    climber_down_c2 = 1
