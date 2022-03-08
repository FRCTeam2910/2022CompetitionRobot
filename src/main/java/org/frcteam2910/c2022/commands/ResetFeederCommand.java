package org.frcteam2910.c2022.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.FeederSubsystem;
import org.frcteam2910.c2022.subsystems.IntakeSubsystem;

public class ResetFeederCommand extends CommandBase {
    private static final double REVERSE_FEEDER_SPEED = -1.0;
    private static final double ONE_BALL_EJECTION_DISTANCE = Units.inchesToMeters(14.0);
    private static final double TWO_BALL_EJECTION_DISTANCE = Units.inchesToMeters(6.0);

    private final FeederSubsystem feeder;
    private final IntakeSubsystem intake;

    private double feederStartingPosition = 0.0;
    private double ejectionDistance = 0.0;

    public ResetFeederCommand(FeederSubsystem feeder, IntakeSubsystem intake) {
        this.feeder = feeder;
        this.intake = intake;

        addRequirements(feeder);
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (feeder.isFull()) {
            ejectionDistance = TWO_BALL_EJECTION_DISTANCE;
        } else {
            ejectionDistance = ONE_BALL_EJECTION_DISTANCE;
        }
        feederStartingPosition = feeder.getPosition();
    }

    @Override
    public void execute() {
        feeder.setFeederSpeed(REVERSE_FEEDER_SPEED);
        intake.setIntakeSpeed(-1.0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(feeder.getPosition() - feederStartingPosition) > ejectionDistance;
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setFeederSpeed(0.0);
        intake.setIntakeSpeed(0.0);
    }
}
