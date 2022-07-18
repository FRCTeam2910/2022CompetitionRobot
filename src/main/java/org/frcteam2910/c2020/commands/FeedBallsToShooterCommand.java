package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;
import org.frcteam2910.c2020.subsystems.IntakeSubsystem;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

public class FeedBallsToShooterCommand extends CommandBase {
    private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> FAST_FEEDER_OUTPUT_MAP = new InterpolatingTreeMap<>();
    private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> SLOW_FEEDER_OUTPUT_MAP = new InterpolatingTreeMap<>();

    private final FeederSubsystem feederSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    static {
        FAST_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(2580.0), new InterpolatingDouble(1.0));
        FAST_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(3873.0), new InterpolatingDouble(1.0));
        FAST_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(4368.0), new InterpolatingDouble(0.7));
        FAST_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(4763.0), new InterpolatingDouble(0.6));
        FAST_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(4965.0), new InterpolatingDouble(0.5));
        FAST_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(5412.0), new InterpolatingDouble(0.5));

        SLOW_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(2580.0), new InterpolatingDouble(1.0));// 0.5 for slow
        SLOW_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(3873.0), new InterpolatingDouble(1.0));
        SLOW_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(4368.0), new InterpolatingDouble(0.7));
        SLOW_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(4763.0), new InterpolatingDouble(0.6));
        SLOW_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(4965.0), new InterpolatingDouble(0.5));
        SLOW_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(5412.0), new InterpolatingDouble(0.5));
    }

    public FeedBallsToShooterCommand(FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem) {
        this.feederSubsystem = feederSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        feederSubsystem.setMoveFirstBallFast(true);
    }

    @Override
    public void execute() {
        if (feederSubsystem.isFull()) {
            feederSubsystem.setMoveFirstBallFast(false);
        }

        if (feederSubsystem.getMoveFirstBallFast()) {// move at 100% speed
            feederSubsystem.spinMotor(FAST_FEEDER_OUTPUT_MAP
                    .getInterpolated(new InterpolatingDouble(shooterSubsystem.getFlywheelVelocity())).value);
        } else {
            feederSubsystem.spinMotor(SLOW_FEEDER_OUTPUT_MAP
                    .getInterpolated(new InterpolatingDouble(shooterSubsystem.getFlywheelVelocity())).value);
        }

        if (feederSubsystem.isFifthBallAtIntake()) {
            intakeSubsystem.setMotorOutput(0.5);
        } else {
            intakeSubsystem.setMotorOutput(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.spinMotor(0.0);
        intakeSubsystem.setMotorOutput(0.0);
    }
}
