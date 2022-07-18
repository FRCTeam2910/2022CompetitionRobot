package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.ClimberSubsystem;

public class DeployClimberCommand extends CommandBase {

    private ClimberSubsystem climberSubsystem;

    public DeployClimberCommand(ClimberSubsystem climber) {
        climberSubsystem = climber;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climberSubsystem.deployClimber();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
