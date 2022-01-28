package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.FeederSubsystem;
import org.frcteam2910.c2022.subsystems.IntakeSubsystem;

public class SimpleIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final FeederSubsystem feeder;
    private double feedSpeed = 0.0;

    public SimpleIntakeCommand(IntakeSubsystem intake, FeederSubsystem feeder, double feedSpeed){
        this.intake = intake;
        this.feeder = feeder;
        this.feedSpeed = feedSpeed;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setExtended(true);
    }

    @Override
    public void execute() {
        feeder.setFeederSpeed(feedSpeed);
        intake.setIntakeSpeed(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setFeederSpeed(0.0);
        intake.setIntakeSpeed(0.0);
    }
}
