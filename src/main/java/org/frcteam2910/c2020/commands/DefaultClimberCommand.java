package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.ClimberSubsystem;

public class DefaultClimberCommand extends CommandBase {
    private final ClimberSubsystem climber;

    public DefaultClimberCommand(ClimberSubsystem climber) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.lock();
        climber.setMotorOutput(0.0);
    }
}
