package org.frcteam2910.c2022;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final int CONTROLLER_PORT = 0;

    public static final int INTAKE_LEFT_MOTOR_PORT = 12;
    public static final int INTAKE_RIGHT_MOTOR_PORT = 13;
    public static final int INTAKE_SOLENOID_PORT = 0;
    public static final int CLIMBER_FIRST_LEFT_MOTOR_PORT = 16;
    public static final int CLIMBER_FIRST_RIGHT_MOTOR_PORT = 11;
    public static final int CLIMBER_SECOND_LEFT_MOTOR_PORT = 17;
    public static final int CLIMBER_SECOND_RIGHT_MOTOR_PORT = 18;
    public static final int FEEDER_MOTOR_PORT = 10;

    public static final int FEEDER_SENSOR_FULL_PORT = 1;
    public static final int FEEDER_SENSOR_ENTRY_PORT = 0;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22);
    public static final int DRIVETRAIN_PIGEON_ID = 1;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 8;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 4;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(90.87 + 180.0);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 3;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(35.59 + 180.0);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 2;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(110.21 + 180.0);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 1;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(78.13 + 180.0);

    public static final int HOOD_MOTOR_PORT = 14;
    public static final int FLYWHEEL_PRIMARY_MOTOR_PORT = 15;
    public static final int FLYWHEEL_SECONDARY_MOTOR_PORT = 9;

    public static final double HOOD_MANUAL_ADJUST_INTERVAL = Math.toRadians(0.5);
    public static final double FLYWHEEL_MANUAL_ADJUST_INTERVAL = Units.rotationsPerMinuteToRadiansPerSecond(25.0);

    public static final double HOOD_MOTOR_TO_HOOD_GEAR_RATIO = 1;
    public static final double HOOD_SHOOTING_ALLOWABLE_ERROR = Math.toRadians(0.5);
    public static final double HOOD_CLIMBING_ALLOWABLE_ERROR = Math.toRadians(1.0);

    public static final String SHOOTER_OFFSET_ENTRY_NAME = "Shooting Offset";
    public static final String HOOD_OFFSET_ENTRY_NAME = "Hood Offset";
    public static final String DRIVER_READOUT_TAB_NAME = "Driver Readout";
}
