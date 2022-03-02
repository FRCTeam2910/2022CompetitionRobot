package org.frcteam2910.c2022.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.FeederSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class FenderShootCommand extends CommandBase {
    private final static double SHOOTER_ANGLE = Math.toRadians(0.0);
    private final static double FLYWHEEL_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(1800);
    private final static double FEEDER_SPEED = 1.0;

    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;

    public FenderShootCommand(FeederSubsystem feeder, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.feeder = feeder;

        addRequirements(feeder, shooter);
    }

    @Override
    public void initialize() {
        shooter.setHoodTargetAngle(SHOOTER_ANGLE);
        shooter.setTargetFlywheelVelocity(FLYWHEEL_SPEED);
    }

    @Override
    public void execute() {
        // To aim when bumpers are flat against the hub wall
        if (shooter.isHoodAtTargetAngle() && shooter.isFlywheelAtTargetSpeed()) {
            feeder.setFeederSpeed(FEEDER_SPEED);
        } else {
            feeder.setFeederSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setFeederSpeed(0.0);
    }
}
