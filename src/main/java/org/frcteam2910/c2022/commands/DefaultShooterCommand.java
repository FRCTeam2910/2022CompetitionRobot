package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase {
    private static double FLYWHEEL_IDLE_SPEED = 1000;

    private final ShooterSubsystem shooter;

    public DefaultShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    public void execute() {
        shooter.setTargetFlywheelSpeed(FLYWHEEL_IDLE_SPEED);
    }
}
