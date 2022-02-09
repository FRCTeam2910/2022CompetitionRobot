package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.FeederSubsystem;
import org.frcteam2910.c2022.subsystems.IntakeSubsystem;

public class ResetFeederCommand extends CommandBase {
    private static final double TIME_TO_REVERSE_FEED = 1.0;
    private static final double REVERSE_FEEDER_SPEED = -0.5;

    private final Timer timer = new Timer();

    private final FeederSubsystem feeder;

    public ResetFeederCommand(FeederSubsystem feeder){
        this.feeder = feeder;

        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        feeder.setFeederSpeed(REVERSE_FEEDER_SPEED);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(TIME_TO_REVERSE_FEED);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setFeederSpeed(0.0);
    }
}
