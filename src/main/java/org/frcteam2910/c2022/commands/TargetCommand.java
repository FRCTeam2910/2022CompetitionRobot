package org.frcteam2910.c2022.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.lib.TargetingSolver;
import org.frcteam2910.c2022.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;
import org.frcteam2910.c2022.util.DriverReadout;
import org.littletonrobotics.subsystems.Vision;

public class TargetCommand extends CommandBase {
    private static final double ROTATION_STATIC_CONSTANT = 0.3;

    private final DrivetrainSubsystem drivetrain;
    private final ShooterSubsystem shooter;
    private final Vision vision;
    private final TargetingSolver targetingSolver;
    private final DriverReadout driverReadout;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;

    public TargetCommand(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, Vision vision,
            TargetingSolver targetingSolver, DriverReadout driverReadout, DoubleSupplier forwardSupplier,
            DoubleSupplier strafeSupplier) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.vision = vision;
        this.targetingSolver = targetingSolver;
        this.driverReadout = driverReadout;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;

        addRequirements(drivetrain, shooter, vision);
    }

    public TargetCommand(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, Vision vision,
            TargetingSolver targetingSolver, DriverReadout driverReadout) {
        this(drivetrain, shooter, vision, targetingSolver, driverReadout, () -> 0.0, () -> 0.0);
    }

    @Override
    public void initialize() {
        vision.setForceLeds(true);

        driverReadout.setTargetPosition(targetingSolver.getTargetPosition());
    }

    @Override
    public void execute() {
        // Get current drivetrain state
        var currentPose = drivetrain.getCurrentPose();
        var currentVelocity = drivetrain.getCurrentVelocity();

        // Determine targeting solution
        var solution = targetingSolver.solve(currentPose.getTranslation(),
                new Translation2d(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond)
                        .rotateBy(currentPose.getRotation()));

        driverReadout.setGoalPosition(solution.targetPosition);

        double forward = forwardSupplier.getAsDouble();
        double strafe = strafeSupplier.getAsDouble();

        drivetrain.setTargetVelocityAndRotation(
                ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, 0.0, currentPose.getRotation()),
                solution.rotation);
    }

    @Override
    public void end(boolean interrupted) {
        vision.setForceLeds(false);
        drivetrain.setTargetVelocity(new ChassisSpeeds());
    }
}
