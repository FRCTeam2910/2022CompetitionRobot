package org.frcteam2910.c2022;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final int CONTROLLER_PORT = 0;

    public static final int INTAKE_MOTOR_PORT = 0;
    public static final int INTAKE_SOLENOID_PORT = 0;
    public static final int CLIMBER_MOTOR_PORT = 0;
    public static final int FEEDER_MOTOR_PORT = 0;
    public static final int FEEDER_SOLENOID_PORT = 0;

    public static final int FEEDER_SENSOR_FULL_PORT = 0;
    public static final int FEEDER_SENSOR_ENTRY_PORT = 0;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22);
    public static final int DRIVETRAIN_PIGEON_ID = 1;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

    public static final int HOOD_MOTOR_PORT = 0;

    public static final double HOOD_MANUAL_ADJUST_INTERVAL = Math.toRadians(0.5);
    public static final double FLYWHEEL_MANUAL_ADJUST_INTERVAL = 50.0;

    public static final double HOOD_MOTOR_TO_HOOD_GEAR_RATIO = 1;
    public static final double HOOD_ALLOWABLE_ERROR = 1.0;
}
