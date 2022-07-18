package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;
import org.frcteam2910.c2020.subsystems.VisionSubsystem;

public class AutonomousFeedCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private FeederSubsystem feederSubsystem;
    private VisionSubsystem visionSubsystem;

    public AutonomousFeedCommand(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem,
            VisionSubsystem visionSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void execute() {
        if (shooterSubsystem.isFlywheelAtTargetVelocity() && visionSubsystem.isOnTarget()) {
            feederSubsystem.spinMotor(0.75);
        }
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.spinMotor(0.0);
    }
}
