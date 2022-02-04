package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.FeederSubsystem;
import org.frcteam2910.c2022.subsystems.IntakeSubsystem;

public class SimpleIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;

    private final double INTAKE_SPEED = 0.5;

    public SimpleIntakeCommand(IntakeSubsystem intake){
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setExtended(true);
    }

    @Override
    public void execute() {
        intake.setIntakeSpeed(INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0.0);
    }
}
