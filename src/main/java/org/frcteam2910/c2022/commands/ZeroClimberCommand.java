package org.frcteam2910.c2022.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ClimberSubsystem;

public class ZeroClimberCommand extends CommandBase {

    private static final double CLIMBER_ZERO_VELOCITY_TIME_PERIOD = 0.5;

    private static final double REVERSE_VOLTAGE = -0.75;
    private static final double VELOCITY_THRESHOLD = Units.inchesToMeters(0.1);

    private final ClimberSubsystem climber;

    private double zeroVelocityTimestamp;

    public ZeroClimberCommand(ClimberSubsystem climber) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        zeroVelocityTimestamp = Double.NaN;
        climber.setZeroed(false);
    }

    @Override
    public void execute() {
        climber.setTargetVoltage(REVERSE_VOLTAGE);
        if (Math.abs(climber.getCurrentVelocity()) < VELOCITY_THRESHOLD) {
            if (!Double.isFinite(zeroVelocityTimestamp)) {
                zeroVelocityTimestamp = Timer.getFPGATimestamp();
            }
        } else {
            zeroVelocityTimestamp = Double.NaN;
        }
    }

    @Override
    public boolean isFinished() {
        if (Double.isFinite(zeroVelocityTimestamp)) {
            return Timer.getFPGATimestamp() - zeroVelocityTimestamp >= CLIMBER_ZERO_VELOCITY_TIME_PERIOD;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climber.setTargetVoltage(0.0);
        if (!interrupted) {
            climber.setZeroPosition();
            climber.setTargetHeight(0);
            climber.setZeroed(true);
        }
    }
}
