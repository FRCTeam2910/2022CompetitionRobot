package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ClimberSubsystem;

public class ClimberToPointCommand extends CommandBase {
    public final ClimberSubsystem climber;
    public double height;

    public ClimberToPointCommand(ClimberSubsystem climber, double height) {
        this.climber = climber;
        this.height = height;
    }

    @Override
    public void initialize() {
        climber.setTargetHeight(height);
    }

    @Override
    public boolean isFinished() {
        return climber.isAtTargetPosition();
    }
}
