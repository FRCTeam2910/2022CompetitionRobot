package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ClimberSubsystem;

public class ClimberToPointCommand extends CommandBase {
    public final ClimberSubsystem climber;
    public double setpoint;

    public ClimberToPointCommand(ClimberSubsystem climber, double setpoint){
        this.climber = climber;
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        climber.setTargetHeight(setpoint);
        climber.setManual(false);
    }

    @Override
    public boolean isFinished() {
        System.out.println(climber.getManual());
        return climber.isAtTargetHeight();
    }
}
