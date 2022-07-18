package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.IntakeSubsystem;

public class ExtendBottomIntakeCommand extends CommandBase {

    private final IntakeSubsystem intake;

    public ExtendBottomIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intake = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intake.setBottomExtended(true);

        // if (Robot.getInstance().isAutonomous()) {
        // intake.setBottomExtended(false);
        // }
        // else {
        // intake.setBottomExtended(true);
        // }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
