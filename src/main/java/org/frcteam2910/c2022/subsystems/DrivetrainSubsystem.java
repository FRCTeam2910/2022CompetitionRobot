package org.frcteam2910.c2022.subsystems;

import java.util.Map;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frcteam2910.c2022.Robot;
import org.frcteam2910.c2022.util.Utilities;
import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;

import static org.frcteam2910.c2022.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
    public static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * SdsModuleConfigurations.MK4_L3.getDriveReduction() * SdsModuleConfigurations.MK4_L3.getWheelDiameter()
            * Math.PI;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(0.891,
            0.15, 0.13592);

    public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
            new FeedforwardConstraint(4.0, FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                    FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false),
            new MaxAccelerationConstraint(5.0), new CentripetalAccelerationConstraint(5.0)};

    private final HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            new PidConstants(5.0, 0.0, 0.0), new PidConstants(5.0, 0.0, 0.0),
            new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
    private final SwerveDrivePoseEstimator estimator;

    private final Pigeon2 pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private double movingDistanceToHubX;
    private double movingDistanceToHubY;

    private boolean onTargetOffset = false;

    private ChassisSpeeds currentVelocity = new ChassisSpeeds();

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private final NetworkTableEntry motorOutputPercentageLimiterEntry;
    private final NetworkTableEntry shootingWhileMovingCoefficientEntry;
    private double motorOutputLimiter;
    private double shootingWhileMovingCoefficient;

    public DrivetrainSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(0, 0),
                Mk4iSwerveModuleHelper.GearRatio.L3, FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER, FRONT_LEFT_MODULE_STEER_OFFSET);
        frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(2, 0),
                Mk4iSwerveModuleHelper.GearRatio.L3, FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER, FRONT_RIGHT_MODULE_STEER_OFFSET);
        backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(4, 0),
                Mk4iSwerveModuleHelper.GearRatio.L3, BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_STEER_OFFSET);
        backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 3).withPosition(6, 0),
                Mk4iSwerveModuleHelper.GearRatio.L3, BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER, BACK_RIGHT_MODULE_STEER_OFFSET);
        estimator = new SwerveDrivePoseEstimator(getGyroscopeRotation(), new Pose2d(), kinematics,
                VecBuilder.fill(0.02, 0.02, 0.01), // estimator values (x, y, rotation) std-devs
                VecBuilder.fill(0.01), // Gyroscope rotation std-dev
                VecBuilder.fill(0.1, 0.1, 0.01)); // Vision (x, y, rotation) std-devs

        motorOutputPercentageLimiterEntry = tab.add("Motor Percentage", 100.0).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 100.0, "Block increment", 10.0)).withPosition(0, 3)
                .getEntry();

        shootingWhileMovingCoefficientEntry = tab.add("Shooting While Moving Coefficient", 1.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 2.0, "Block increment", 0.1)).withPosition(2, 3).getEntry();

        tab.addNumber("Odometry X", () -> getPose().getX());
        tab.addNumber("Odometry Y", () -> getPose().getY());
        tab.addNumber("Odometry Angle", () -> getPose().getRotation().getDegrees());
        tab.addNumber("Velocity X", () -> Units.metersToFeet(getCurrentVelocity().vxMetersPerSecond));
        tab.addNumber("Trajectory Position X", () -> {
            var lastState = follower.getLastState();
            if (lastState == null)
                return 0;

            return Units.metersToFeet(lastState.getPathState().getPosition().x);
        });
        tab.addNumber("Trajectory Velocity X", () -> {
            var lastState = follower.getLastState();
            if (lastState == null)
                return 0;

            return Units.metersToFeet(lastState.getVelocity());
        });
        tab.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees());
        tab.addNumber("Moving distance offset x", () -> getHubDistanceMovingOffset().x);
        tab.addNumber("Moving distance offset y", () -> getHubDistanceMovingOffset().y);
        tab.addBoolean("Is on target", this::isOnTargetOffset);
        tab.addNumber("Angle to target",
                () -> Math.toDegrees(Math.atan2((getPose().getY() * -1) + getHubDistanceMovingOffset().y,
                        (getPose().getX() * -1) + getHubDistanceMovingOffset().x)));

        // pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 255);
        // pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel,
        // 255);
        pigeon.configFactoryDefault();
    }

    private Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    /**
     * Resets the rotation of the drivetrain to zero.
     */
    public void zeroRotation() {
        estimator.resetPosition(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d()),
                getGyroscopeRotation());
    }

    /**
     * Returns the position of the robot
     */
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public double getMotorOutputLimiter() {
        return motorOutputLimiter;
    }

    public HolonomicMotionProfiledTrajectoryFollower getFollower() {
        return follower;
    }

    public ChassisSpeeds getCurrentVelocity() {
        return currentVelocity;
    }

    public boolean isOnTargetOffset() {
        return onTargetOffset;
    }

    public void setOnTargetOffset(boolean onTargetOffset) {
        this.onTargetOffset = onTargetOffset;
    }

    /**
     * Sets the position of the robot to the position passed in with the current
     * gyroscope rotation.
     */
    public void setPose(Pose2d pose) {
        estimator.resetPosition(pose, getGyroscopeRotation());
    }

    /**
     * Sets the desired chassis speed of the drivetrain.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    public void periodic() {
        SwerveModuleState currentFrontLeftModuleState = new SwerveModuleState(frontLeftModule.getDriveVelocity(),
                new Rotation2d(frontLeftModule.getSteerAngle()));
        SwerveModuleState currentFrontRightModuleState = new SwerveModuleState(frontRightModule.getDriveVelocity(),
                new Rotation2d(frontRightModule.getSteerAngle()));
        SwerveModuleState currentBackLeftModuleState = new SwerveModuleState(backLeftModule.getDriveVelocity(),
                new Rotation2d(backLeftModule.getSteerAngle()));
        SwerveModuleState currentBackRightModuleState = new SwerveModuleState(backRightModule.getDriveVelocity(),
                new Rotation2d(backRightModule.getSteerAngle()));

        currentVelocity = kinematics.toChassisSpeeds(currentFrontLeftModuleState, currentFrontRightModuleState,
                currentBackLeftModuleState, currentBackRightModuleState);

        estimator.update(getGyroscopeRotation(), currentFrontLeftModuleState, currentFrontRightModuleState,
                currentBackLeftModuleState, currentBackRightModuleState);

        var driveSignalOpt = follower.update(Utilities.poseToRigidTransform(getPose()),
                new Vector2(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond),
                currentVelocity.omegaRadiansPerSecond, Timer.getFPGATimestamp(), Robot.kDefaultPeriod);

        if (driveSignalOpt.isPresent()) {
            HolonomicDriveSignal driveSignal = driveSignalOpt.get();
            if (driveSignalOpt.get().isFieldOriented()) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveSignal.getTranslation().x,
                        driveSignal.getTranslation().y, driveSignal.getRotation(), getPose().getRotation());
            } else {
                chassisSpeeds = new ChassisSpeeds(driveSignal.getTranslation().x, driveSignal.getTranslation().y,
                        driveSignal.getRotation());
            }
        }

        motorOutputLimiter = motorOutputPercentageLimiterEntry.getDouble(0.0) / 100;
        shootingWhileMovingCoefficient = shootingWhileMovingCoefficientEntry.getDouble(1.0);

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());

        movingDistanceToHubX = shootingWhileMovingCoefficient * chassisSpeeds.vxMetersPerSecond;
        movingDistanceToHubY = shootingWhileMovingCoefficient * chassisSpeeds.vyMetersPerSecond;
    }

    public Vector2 getHubDistanceMovingOffset() {
        return new Vector2(movingDistanceToHubX, movingDistanceToHubY);
    }
}