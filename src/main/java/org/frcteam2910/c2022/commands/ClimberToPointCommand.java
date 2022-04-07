package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ClimberSubsystem;

public class ClimberToPointCommand extends CommandBase {
    public final ClimberSubsystem climber;
    public double height;
    public boolean fast;

    public ClimberToPointCommand(ClimberSubsystem climber, double height) {
        this(climber, height, true);
    }

    public ClimberToPointCommand(ClimberSubsystem climber, double height, boolean fast) {
        this.climber = climber;
        this.height = height;
        this.fast = fast;
    }

    @Override
    public void initialize() {
        climber.setTargetHeight(height);
        climber.setFastClimberConstraints(fast);
    }

    @Override
    public boolean isFinished() {
        return climber.isAtTargetPosition();
    }
}
