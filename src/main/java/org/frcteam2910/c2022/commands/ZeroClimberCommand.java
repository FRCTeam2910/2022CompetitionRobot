package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ClimberSubsystem;

public class ZeroClimberCommand extends CommandBase {

    private static final long CLIMBER_ZERO_VELOCITY_TIME_PERIOD = 250;

    private final ClimberSubsystem climber;

    private double zeroVelocityTimestamp;

    public ZeroClimberCommand(ClimberSubsystem climber){
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        zeroVelocityTimestamp = Long.MAX_VALUE;
    }

    @Override
    public void execute() {
        climber.setMotorSpeed(-0.1);
        if(Math.abs(climber.getVelocity()) < 0.05){
            if(zeroVelocityTimestamp == Long.MAX_VALUE) {
                zeroVelocityTimestamp = Timer.getFPGATimestamp();
            }
        } else {
            zeroVelocityTimestamp = Long.MAX_VALUE;
        }
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - zeroVelocityTimestamp >= CLIMBER_ZERO_VELOCITY_TIME_PERIOD;
    }

    @Override
    public void end(boolean interrupted) {
        climber.setMotorSpeed(0.0);
        climber.zeroClimber();
    }
}
