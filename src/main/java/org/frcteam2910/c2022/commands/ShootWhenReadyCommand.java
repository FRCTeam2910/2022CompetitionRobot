package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2022.subsystems.FeederSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;
import org.frcteam2910.c2022.subsystems.VisionSubsystem;

public class ShootWhenReadyCommand extends CommandBase {
    private static final double FEEDER_SPEED = 1.0;

    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;

    public ShootWhenReadyCommand(FeederSubsystem feeder, ShooterSubsystem shooter, VisionSubsystem vision,
            DrivetrainSubsystem drivetrain) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.vision = vision;
        this.drivetrain = drivetrain;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        if (drivetrain.isOnTargetOffset() & shooter.isFlywheelAtTargetSpeed() & shooter.isHoodAtTargetAngle()) {
            feeder.setFeederSpeed(FEEDER_SPEED);
        } else {
            feeder.setFeederSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTargetFlywheelSpeed(0.0);
    }
}
