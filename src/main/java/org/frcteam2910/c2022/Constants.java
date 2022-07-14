package org.frcteam2910.c2022;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final int CONTROLLER_PORT = 0;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22);
    public static final int DRIVETRAIN_PIGEON_ID = 0;

    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 14;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 3;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 8;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 9;

    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 4;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 1;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 10;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 7;

    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 15;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 14;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 16;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 17;

    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(258.39 - 180.0);
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(315.09 - 180.0);
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(122.52 + 180.0);
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(222.01 - 180.0);
}
