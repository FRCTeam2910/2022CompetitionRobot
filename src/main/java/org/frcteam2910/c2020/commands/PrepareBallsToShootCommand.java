package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;

public class PrepareBallsToShootCommand extends CommandBase {
    private final FeederSubsystem feeder;
    private final double speed;

    public PrepareBallsToShootCommand(FeederSubsystem feeder, double speed) {
        this.feeder = feeder;
        this.speed = speed;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        feeder.spinMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.spinMotor(0.0);
    }

    @Override
    public boolean isFinished() {
        return feeder.isFull();
    }
}
