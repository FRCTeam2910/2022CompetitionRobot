package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.Superstructure;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;
import org.frcteam2910.c2020.subsystems.IntakeSubsystem;

public class RetractIntakeCommand extends CommandBase {
    private final FeederSubsystem feeder;
    private final IntakeSubsystem intake;
    private final Superstructure superstructure;

    public RetractIntakeCommand(FeederSubsystem feeder, IntakeSubsystem intake, Superstructure superstructure) {
        this.feeder = feeder;
        this.intake = intake;
        this.superstructure = superstructure;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setMotorOutput(-1.0);

        if (superstructure.getCurrentPressure() > IntakeSubsystem.MIN_INTAKE_MOVEMENT_PRESSURE
                && !feeder.isFifthBallAtIntake()) {
            intake.setTopExtended(false);
        }
    }

    @Override
    public boolean isFinished() {
        return feeder.isFifthBallAtIntake();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotorOutput(0.0);
    }
}
