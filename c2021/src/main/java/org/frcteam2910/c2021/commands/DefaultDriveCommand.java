package org.frcteam2910.c2021.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2021.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrain, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        drivetrain.setTargetVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble(), rotationSupplier.getAsDouble(),
                drivetrain.getCurrentPose().getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setTargetVelocity(new ChassisSpeeds());
    }
}
