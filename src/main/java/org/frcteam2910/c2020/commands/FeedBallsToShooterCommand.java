package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

public class FeedBallsToShooterCommand extends CommandBase {
    private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> FEEDER_OUTPUT_MAP = new InterpolatingTreeMap<>();

    private final FeederSubsystem feederSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    static {
        FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(4700.0), new InterpolatingDouble(1.0));
        FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(5000.0), new InterpolatingDouble(0.7));
        FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(5500.0), new InterpolatingDouble(0.5));
        FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(5700.0), new InterpolatingDouble(0.4));
    }

    public FeedBallsToShooterCommand(FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem) {
        this.feederSubsystem = feederSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        feederSubsystem.spinMotor(FEEDER_OUTPUT_MAP
                .getInterpolated(new InterpolatingDouble(shooterSubsystem.getFlywheelTargetVelocity())).value);
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.spinMotor(0.0);
    }
}
