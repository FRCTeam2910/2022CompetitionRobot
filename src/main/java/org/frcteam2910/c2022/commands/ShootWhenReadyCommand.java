package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2022.subsystems.FeederSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class ShootWhenReadyCommand extends CommandBase {
    private static final double FEEDER_SPEED = 1.0;

    private final DrivetrainSubsystem drivetrain;
    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;
    private final Timer timer = new Timer();

    public ShootWhenReadyCommand(DrivetrainSubsystem drivetrain, FeederSubsystem feeder, ShooterSubsystem shooter) {
        this.drivetrain = drivetrain;
        this.feeder = feeder;
        this.shooter = shooter;

        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        if (drivetrain.isRotationAtTarget() & shooter.isFlywheelAtTargetSpeed() & shooter.isHoodAtTargetAngle()) {
            feeder.setFeederSpeed(FEEDER_SPEED);
            timer.reset();
        } else {
            feeder.setFeederSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTargetFlywheelSpeed(0.0);
    }
}
