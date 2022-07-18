package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final double flywheelRpm;
    private final double hoodAngle;

    public DefaultShooterCommand(ShooterSubsystem shooter, double flywheelRpm, double hoodAngle) {
        this.shooter = shooter;
        this.flywheelRpm = flywheelRpm;
        this.hoodAngle = hoodAngle;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.shootFlywheel(flywheelRpm);
        shooter.setHoodTargetAngle(hoodAngle);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
    }
}
