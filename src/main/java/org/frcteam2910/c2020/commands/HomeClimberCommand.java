package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.Superstructure;
import org.frcteam2910.c2020.subsystems.ClimberSubsystem;

public class HomeClimberCommand extends CommandBase {

    private static final long CLIMBER_ZERO_VELOCITY_TIME_PERIOD = 250;
    private static final long CLIMBER_TIME_TO_UNLOCK = 500;

    private final ClimberSubsystem climber;
    private final Superstructure superstructure;

    private long timeToUnlock;
    private long zeroVelocityTimestamp;

    public HomeClimberCommand(ClimberSubsystem climber, Superstructure superstructure) {
        this.climber = climber;
        this.superstructure = superstructure;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.unlock();
        timeToUnlock = System.currentTimeMillis();
        zeroVelocityTimestamp = Long.MAX_VALUE;
        climber.disableSoftLimits();
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() - timeToUnlock >= CLIMBER_TIME_TO_UNLOCK) {
            climber.setMotorOutput(-0.4);
        }

        if (Math.abs(climber.getClimberVelocity()) < 50
                && System.currentTimeMillis() - timeToUnlock >= CLIMBER_TIME_TO_UNLOCK) {
            if (zeroVelocityTimestamp == Long.MAX_VALUE) {
                zeroVelocityTimestamp = System.currentTimeMillis();
            }
        } else {
            zeroVelocityTimestamp = Long.MAX_VALUE;
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - zeroVelocityTimestamp >= CLIMBER_ZERO_VELOCITY_TIME_PERIOD;
    }

    @Override
    public void end(boolean interrupted) {
        climber.setMotorOutput(0.0);
        climber.lock();
        climber.zeroHeight();
        climber.enableSoftLimits();
    }
}
