package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.Superstructure;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;
import org.frcteam2910.c2020.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;
    private FeederSubsystem feederSubsystem;
    private final Superstructure superstructure;

    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem,
            Superstructure superstructure) {
        this.intakeSubsystem = intakeSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.superstructure = superstructure;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setBottomExtended(true);// Can remove this; just make sure to make the field true in the
                                                // subsystem

        if (feederSubsystem.isFifthBallAtIntake()) {
            intakeSubsystem.setTopExtended(true);
        } else {
            if (superstructure.getCurrentPressure() >= IntakeSubsystem.MIN_INTAKE_MOVEMENT_PRESSURE) {
                intakeSubsystem.setTopExtended(false);
            }
        }

        // If a fifth ball is no longer in the intake, put the intake up.
        // This is used to override the
        // "intake.setTopExtended(feederSubsystem.isFifthBallAtIntake());" in the
        // SimpleIntakeCommand if the fifth ball is no longer in the system, and thus
        // the top intake must go up
        // intakeSubsystem.setTopExtended(feederSubsystem.isFifthBallAtIntake());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
