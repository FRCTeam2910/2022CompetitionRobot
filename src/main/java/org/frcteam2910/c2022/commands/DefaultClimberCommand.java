package org.frcteam2910.c2022.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ClimberSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultClimberCommand extends CommandBase {
    public final ClimberSubsystem climber;
    public DoubleSupplier joystick;

    public DefaultClimberCommand(ClimberSubsystem climber, DoubleSupplier joystick){
        this.climber = climber;
        this.joystick = joystick;
        addRequirements(climber);
    }

    @Override
    public void execute() { 
        if(Math.abs(joystick.getAsDouble()) > 0.1){
            climber.setManual(true);
        }
        if(climber.getManual()) {
            if(climber.getPositionControl()) {
                if (-joystick.getAsDouble() > 0) {
                    climber.setMotorSpeed(-joystick.getAsDouble() + 0.1);
                } else {
                    climber.setMotorSpeed(0.1);
                }
            } else {
                if (Math.abs(joystick.getAsDouble()) > 0.1) {
                    climber.setTargetVelocity(-joystick.getAsDouble() * ClimberSubsystem.MAX_VELOCITY);
                } else {
                    climber.setTargetVelocity(0.0);
                }
            }
        }
    }
}
