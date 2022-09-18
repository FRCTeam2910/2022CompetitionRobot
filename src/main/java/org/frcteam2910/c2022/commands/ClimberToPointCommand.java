package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ClimberSubsystem;

public class ClimberToPointCommand extends CommandBase {
    public final ClimberSubsystem climber;
    public double height;
    public boolean fast;
    public boolean evenFaster;

    public ClimberToPointCommand(ClimberSubsystem climber, double height) {
        this(climber, height, true, false);
    }

    public ClimberToPointCommand(ClimberSubsystem climber, double height, boolean fast, boolean evenFaster) {
        this.climber = climber;
        this.height = height;
        this.fast = fast;
        this.evenFaster = evenFaster;
    }

    @Override
    public void initialize() {
        climber.setTargetHeight(height);
        if (evenFaster) {
            climber.setEvenFasterMotionConstraints();
        } else {
            climber.setFastClimberConstraints(fast);
        }
    }

    @Override
    public boolean isFinished() {
        return climber.isAtTargetPosition();
    }
}
