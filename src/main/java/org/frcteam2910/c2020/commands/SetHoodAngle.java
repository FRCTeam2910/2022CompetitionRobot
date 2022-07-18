package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;

public class SetHoodAngle extends CommandBase {
    private ShooterSubsystem shooter;
    private double targetAngle;
    public SetHoodAngle(ShooterSubsystem shooter, double targetAngle) {
        this.shooter = shooter;
        this.targetAngle = targetAngle;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setHoodTargetAngle(targetAngle);
    }
}
