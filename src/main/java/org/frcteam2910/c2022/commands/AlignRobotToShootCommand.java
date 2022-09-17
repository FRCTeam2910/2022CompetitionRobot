package org.frcteam2910.c2022.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.Robot;
import org.frcteam2910.c2022.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2022.subsystems.VisionSubsystem;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;

public class AlignRobotToShootCommand extends CommandBase {
    private static final double ROTATION_STATIC_CONSTANT = 0.3;
    private static final double TARGET_ALLOWABLE_ERROR = Math.toRadians(2.5);

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final PidController controller = new PidController(new PidConstants(5.0, 0.0, 0.3));
    private boolean targetSeen = false;

    private final DoubleSupplier xAxis;
    private final DoubleSupplier yAxis;

    public AlignRobotToShootCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, DoubleSupplier xAxis,
            DoubleSupplier yAxis) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.xAxis = xAxis;
        this.yAxis = yAxis;

        controller.setInputRange(0, 2 * Math.PI);
        controller.setContinuous(true);

        addRequirements(drivetrain);
    }

    public AlignRobotToShootCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        this(drivetrain, vision, () -> 0.0, () -> 0.0);
    }

    @Override
    public void initialize() {
        controller.reset();
        drivetrain.setOnTargetOffset(false);
    }

    @Override
    public void execute() {
        if (targetSeen) {
            Rotation2d currentAngle = drivetrain.getPose().getRotation();

            double distanceX = (drivetrain.getPose().getX() * -1) - drivetrain.getHubDistanceMovingOffset().x;
            double distanceY = (drivetrain.getPose().getY() * -1) - drivetrain.getHubDistanceMovingOffset().y;
            double angle = Math.atan2(distanceY, distanceX);

            // if(Math.hypot(drivetrain.getHubDistanceMovingOffset().x,
            // drivetrain.getHubDistanceMovingOffset().y) > 99.1){
            // controller.setSetpoint(angle);
            // double rotationalVelocity = controller.calculate(currentAngle.getRadians(),
            // Robot.kDefaultPeriod);
            // rotationalVelocity += Math.copySign(ROTATION_STATIC_CONSTANT /
            // DrivetrainSubsystem.MAX_VOLTAGE
            // * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            // rotationalVelocity);
            // drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xAxis.getAsDouble(),
            // yAxis.getAsDouble(),
            // rotationalVelocity, currentAngle));
            // drivetrain.setOnTargetOffset(MathUtils.epsilonEquals(angle,
            // drivetrain.getPose().getRotation().getRadians(),
            // TARGET_ALLOWABLE_ERROR));
            // } else {
            controller.setSetpoint(vision.getAngleToTarget());
            double rotationalVelocity = controller.calculate(currentAngle.getRadians(), Robot.kDefaultPeriod);
            rotationalVelocity += Math.copySign(ROTATION_STATIC_CONSTANT / DrivetrainSubsystem.MAX_VOLTAGE
                    * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, rotationalVelocity);
            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xAxis.getAsDouble(), yAxis.getAsDouble(),
                    -rotationalVelocity, currentAngle));
            drivetrain.setOnTargetOffset(vision.isOnTarget());
            // }
            if (vision.isOnTarget()) {
                double distanceToTarget = Math.hypot(vision.getDistanceToTarget().x, vision.getDistanceToTarget().y);
                drivetrain.setPose(new Pose2d(distanceToTarget * Math.cos(currentAngle.getRadians()) * -1,
                        distanceToTarget * Math.sin(currentAngle.getRadians()) * -1,
                        drivetrain.getPose().getRotation()));
            }
        } else {
            targetSeen = vision.shooterHasTargets();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}