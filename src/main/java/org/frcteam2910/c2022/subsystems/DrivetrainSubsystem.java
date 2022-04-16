package org.frcteam2910.c2022.subsystems;

import java.util.Optional;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
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
import org.frcteam2910.c2022.Robot;
import org.frcteam2910.c2022.lib.IDrivetrain;
import org.frcteam2910.c2022.lib.wpilib.SwerveDrivePoseEstimator;
import org.frcteam2910.c2022.lib.wpilib.TimeInterpolatableBuffer;
import org.frcteam2910.c2022.util.Utilities;
import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;

import static org.frcteam2910.c2022.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase implements IDrivetrain {
    public static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * SdsModuleConfigurations.MK4_L3.getDriveReduction() * SdsModuleConfigurations.MK4_L3.getWheelDiameter()
            * Math.PI;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final double ROTATION_STATIC_CONSTANT = 0.3;

    public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(0.891,
            0.15, 0.13592);

    public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
            new FeedforwardConstraint(4.0, FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                    FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false),
            new MaxAccelerationConstraint(5.0), new CentripetalAccelerationConstraint(5.0)};

    private final HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            new PidConstants(5.0, 0.0, 0.0), new PidConstants(5.0, 0.0, 0.0),
            new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));
    private final PIDController rotationController = new PIDController(5.0, 0.0, 0.3);

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
    private final TimeInterpolatableBuffer<Pose2d> previousPoses = TimeInterpolatableBuffer.createBuffer(0.5);

    private final Pigeon2 pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds currentVelocity = new ChassisSpeeds();

    private ChassisSpeeds targetVelocity = new ChassisSpeeds();
    private Rotation2d targetRotation = null;

    public DrivetrainSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                Mk4iSwerveModuleHelper.GearRatio.L3, FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER, FRONT_LEFT_MODULE_STEER_OFFSET);
        frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
                Mk4iSwerveModuleHelper.GearRatio.L3, FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER, FRONT_RIGHT_MODULE_STEER_OFFSET);
        backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
                Mk4iSwerveModuleHelper.GearRatio.L3, BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_STEER_OFFSET);
        backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
                Mk4iSwerveModuleHelper.GearRatio.L3, BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER, BACK_RIGHT_MODULE_STEER_OFFSET);
        estimator = new SwerveDrivePoseEstimator(getGyroscopeRotation(), new Pose2d(), kinematics,
                VecBuilder.fill(0.02, 0.02, 0.01), // estimator values (x, y, rotation) std-devs
                VecBuilder.fill(0.01), // Gyroscope rotation std-dev
                VecBuilder.fill(0.1, 0.1, 0.01)); // Vision (x, y, rotation) std-devs

        tab.addNumber("Odometry X", () -> Units.metersToFeet(getCurrentPose().getX()));
        tab.addNumber("Odometry Y", () -> Units.metersToFeet(getCurrentPose().getY()));
        tab.addNumber("Odometry Angle", () -> getCurrentPose().getRotation().getDegrees());
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

    public HolonomicMotionProfiledTrajectoryFollower getFollower() {
        return follower;
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

        var driveSignalOpt = follower.update(Utilities.poseToRigidTransform(getCurrentPose()),
                new Vector2(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond),
                currentVelocity.omegaRadiansPerSecond, Timer.getFPGATimestamp(), Robot.kDefaultPeriod);

        if (driveSignalOpt.isPresent()) {
            HolonomicDriveSignal driveSignal = driveSignalOpt.get();
            if (driveSignalOpt.get().isFieldOriented()) {
                targetVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(driveSignal.getTranslation().x,
                        driveSignal.getTranslation().y, driveSignal.getRotation(), getCurrentPose().getRotation());
            } else {
                targetVelocity = new ChassisSpeeds(driveSignal.getTranslation().x, driveSignal.getTranslation().y,
                        driveSignal.getRotation());
            }
        } else if (targetRotation != null) {
            double rotationError = MathUtil
                    .angleModulus(getCurrentPose().getRotation().getRadians() - targetRotation.getRadians());
            targetVelocity.omegaRadiansPerSecond = rotationController.calculate(rotationError, 0.0);
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