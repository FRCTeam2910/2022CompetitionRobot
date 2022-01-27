package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.IntakeSubsystem;

public class ResetIntakeCommand extends CommandBase {
    public final IntakeSubsystem intake;
    private final Timer timer = new Timer();
    private final double TIME_TO_REVERSE_FEED = 1.0;

    public ResetIntakeCommand(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setFeederSpeed(-1.0);
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(TIME_TO_REVERSE_FEED);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setExtended(false);
    }
}
