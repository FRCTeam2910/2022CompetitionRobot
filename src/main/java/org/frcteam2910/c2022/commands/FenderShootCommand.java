package org.frcteam2910.c2022.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.FeederSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class FenderShootCommand extends CommandBase {
    private final static double SHOOTER_ANGLE = Math.toRadians(87);
    private final static double FLYWHEEL_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(3000);
    private final static double FEEDER_SPEED = 0.5;

    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;

    public FenderShootCommand(FeederSubsystem feeder, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.feeder = feeder;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        // To aim when bumpers are flat against the hub wall
        shooter.setHoodTargetPosition(SHOOTER_ANGLE);
        shooter.setTargetFlywheelSpeed(FLYWHEEL_SPEED);
        feeder.setFeederSpeed(FEEDER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTargetFlywheelSpeed(0.0);
        feeder.setFeederSpeed(0.0);
    }
}
