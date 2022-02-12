package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ClimberSubsystem;

public class ZeroClimberCommand extends CommandBase {

    private static final long CLIMBER_ZERO_VELOCITY_TIME_PERIOD = 250;

    private static final double REVERSE_VOLTAGE = -2.0;
    private static final double VELOCITY_THRESHOLD = 0.1;

    private final ClimberSubsystem climber;

    private double zeroVelocityTimestamp;

    public ZeroClimberCommand(ClimberSubsystem climber) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        zeroVelocityTimestamp = Double.NaN;
    }

    @Override
    public void execute() {
        climber.setTargetVoltage(REVERSE_VOLTAGE);
        if (Math.abs(climber.getCurrentVelocity()) < VELOCITY_THRESHOLD) {
            if (Double.isFinite(zeroVelocityTimestamp)) {
                zeroVelocityTimestamp = Timer.getFPGATimestamp();
            }
        } else {
            zeroVelocityTimestamp = Double.NaN;
        }
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - zeroVelocityTimestamp >= CLIMBER_ZERO_VELOCITY_TIME_PERIOD;
    }

    @Override
    public void end(boolean interrupted) {
        climber.setTargetVoltage(0.0);
        if (!interrupted) {
            climber.setZeroPosition();
        }
    }
}
