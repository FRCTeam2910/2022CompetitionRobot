package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;
import org.frcteam2910.c2020.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final FeederSubsystem feeder;
    private final double intakeSpeed;
    private final boolean ignoreFull;

    public IntakeCommand(IntakeSubsystem intake, FeederSubsystem feeder, double intakeSpeed) {
        this(intake, feeder, intakeSpeed, false);
    }

    public IntakeCommand(IntakeSubsystem intake, FeederSubsystem feeder, double intakeSpeed, boolean ignoreFull) {
        this.intake = intake;
        this.feeder = feeder;
        this.intakeSpeed = intakeSpeed;
        this.ignoreFull = ignoreFull;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setTopExtended(true);
        intake.setMotorOutput(intakeSpeed);
    }

    @Override
    public boolean isFinished() {
        return feeder.isFull() && !ignoreFull;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setTopExtended(false);
        intake.setMotorOutput(0.0);
    }
}
