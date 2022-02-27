package org.frcteam2910.c2022.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase {
    private static final double FLYWHEEL_IDLE_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(2000);

    private final ShooterSubsystem shooter;

    public DefaultShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    public void initialize() {
        shooter.setHoodTargetPosition(Math.toRadians(0.0));
        shooter.setTargetFlywheelSpeed(FLYWHEEL_IDLE_SPEED);
    }
}
