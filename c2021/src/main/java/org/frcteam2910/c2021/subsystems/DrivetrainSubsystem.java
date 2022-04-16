package org.frcteam2910.c2021.subsystems;

import java.util.Optional;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frcteam2910.visionlib.IDrivetrain;
import org.frcteam2910.visionlib.wpilib.SwerveDrivePoseEstimator;
import org.frcteam2910.visionlib.wpilib.TimeInterpolatableBuffer;

import static org.frcteam2910.c2021.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase implements IDrivetrain {
    public static final double TRACKWIDTH_METERS = Units.inchesToMeters(22.0);
    public static final double WHEELBASE_METERS = Units.inchesToMeters(22.0);

    public static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * SdsModuleConfigurations.MK4_L3.getDriveReduction() * SdsModuleConfigurations.MK4_L3.getWheelDiameter()
            * Math.PI;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    public static final double ROTATION_STATIC_CONSTANT = 0.3;

    private final PIDController rotationController = new PIDController(5.0, 0.0, 0.3);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));
    private final SwerveDrivePoseEstimator estimator;
    private final TimeInterpolatableBuffer<Pose2d> previousPoses = TimeInterpolatableBuffer.createBuffer(0.5);

    private final Pigeon2 pigeon = new Pigeon2(0);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds currentVelocity = new ChassisSpeeds();

    private ChassisSpeeds targetVelocity = new ChassisSpeeds();
    private Rotation2d targetRotation = null;

    public DrivetrainSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                Mk4SwerveModuleHelper.GearRatio.L4, DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
                DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, DRIVETRAIN_FRONT_LEFT_ENCODER_PORT,
                DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET);
        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
                Mk4SwerveModuleHelper.GearRatio.L4, DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
                DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
                DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET);
        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
                Mk4SwerveModuleHelper.GearRatio.L4, DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
                DRIVETRAIN_BACK_LEFT_ENCODER_PORT, DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET);
        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
                Mk4SwerveModuleHelper.GearRatio.L4, DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
                DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, DRIVETRAIN_BACK_RIGHT_ENCODER_PORT,
                DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET);
        estimator = new SwerveDrivePoseEstimator(getGyroscopeRotation(), new Pose2d(), kinematics,
                VecBuilder.fill(0.02, 0.02, 0.01), // estimator values (x, y, rotation) std-devs
                VecBuilder.fill(0.01), // Gyroscope rotation std-dev
                VecBuilder.fill(0.1, 0.1, 0.01)); // Vision (x, y, rotation) std-devs

        tab.addNumber("Odometry X", () -> Units.metersToFeet(getCurrentPose().getX()));
        tab.addNumber("Odometry Y", () -> Units.metersToFeet(getCurrentPose().getY()));
        tab.addNumber("Odometry Angle", () -> getCurrentPose().getRotation().getDegrees());
        tab.addNumber("Velocity X", () -> Units.metersToFeet(getCurrentVelocity().vxMetersPerSecond));
        tab.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees());

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
        estimator.resetPosition(new Pose2d(getCurrentPose().getX(), getCurrentPose().getY(), new Rotation2d()),
                getGyroscopeRotation());
    }

    @Override
    public Pose2d getCurrentPose() {
        return estimator.getEstimatedPosition();
    }

    @Override
    public Optional<Pose2d> getPreviousPose(double timestamp) {
        return previousPoses.getSample(timestamp);
    }

    @Override
    public ChassisSpeeds getCurrentVelocity() {
        return currentVelocity;
    }

    @Override
    public Optional<Rotation2d> getTargetRotation() {
        return Optional.ofNullable(targetRotation);
    }

    /**
     * Sets the position of the robot to the position passed in with the current
     * gyroscope rotation.
     */
    public void resetPose(Pose2d pose) {
        estimator.resetPosition(pose, getGyroscopeRotation());
        previousPoses.clear();
        previousPoses.addSample(Timer.getFPGATimestamp(), pose);
    }

    /**
     * Sets the desired chassis speed of the drivetrain.
     */
    @Override
    public void setTargetVelocity(ChassisSpeeds chassisSpeeds) {
        this.targetVelocity = chassisSpeeds;
        this.targetRotation = null;
    }

    @Override
    public void setTargetVelocityAndRotation(ChassisSpeeds targetVelocity, Rotation2d targetRotation) {
        this.targetVelocity = targetVelocity;
        if (this.targetRotation == null) {
            rotationController.reset();
        }
        this.targetRotation = targetRotation;
    }

    @Override
    public void addVisionMeasurement(double captureTimestamp, Pose2d pose) {
        estimator.addVisionMeasurement(pose, captureTimestamp);
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
        previousPoses.addSample(Timer.getFPGATimestamp(), estimator.getEstimatedPosition());

        if (targetRotation != null) {
            double rotationError = MathUtil
                    .angleModulus(getCurrentPose().getRotation().getRadians() - targetRotation.getRadians());
            targetVelocity.omegaRadiansPerSecond = rotationController.calculate(rotationError, 0.0);
            targetVelocity.omegaRadiansPerSecond += Math.copySign(ROTATION_STATIC_CONSTANT,
                    targetVelocity.omegaRadiansPerSecond) / MAX_VOLTAGE * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetVelocity);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());
    }
}