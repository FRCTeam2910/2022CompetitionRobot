package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.FeederSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;
import org.frcteam2910.c2022.subsystems.VisionSubsystem;

public class ShootWhenReadyCommand extends CommandBase {
    private static final double FEEDER_SPEED = 1.0;

    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final Timer timer = new Timer();

    public ShootWhenReadyCommand(FeederSubsystem feeder, ShooterSubsystem shooter, VisionSubsystem vision) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.vision = vision;

        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        if (vision.isOnTarget() & shooter.isFlywheelAtTargetSpeed() & shooter.isHoodAtTargetAngle()) {
            feeder.setFeederSpeed(FEEDER_SPEED);
            timer.reset();
        } else {
            feeder.setFeederSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTargetFlywheelVelocity(0.0);
    }
}
