package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class SetHoodAngleCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final double targetAngle;

    public SetHoodAngleCommand(ShooterSubsystem shooter, double targetAngle) {
        this.shooter = shooter;
        this.targetAngle = targetAngle;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setHoodTargetPosition(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return shooter.isHoodAtTargetAngle();
    }
}