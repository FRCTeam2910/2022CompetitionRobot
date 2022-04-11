package org.frcteam2910.c2022.commands;

import java.util.function.DoubleSupplier;

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
    }

    @Override
    public void execute() {
        if (targetSeen) {
            Rotation2d currentAngle = drivetrain.getPose().getRotation();

            controller.setSetpoint(
                    vision.getAngleToTarget() - (drivetrain.getHubRotationMovingOffset() - vision.getAngleToTarget()));
            double rotationalVelocity = controller.calculate(currentAngle.getRadians(), Robot.kDefaultPeriod);
            rotationalVelocity += Math.copySign(ROTATION_STATIC_CONSTANT / DrivetrainSubsystem.MAX_VOLTAGE
                    * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, rotationalVelocity);
            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xAxis.getAsDouble(), yAxis.getAsDouble(),
                    -rotationalVelocity, currentAngle));
        } else {
            targetSeen = vision.shooterHasTargets();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}