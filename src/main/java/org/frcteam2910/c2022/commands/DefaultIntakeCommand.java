package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;

    public DefaultIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setExtended(false);
    }
}
