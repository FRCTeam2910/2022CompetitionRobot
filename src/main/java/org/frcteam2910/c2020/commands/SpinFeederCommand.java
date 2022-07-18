package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;

public class SpinFeederCommand extends CommandBase {

    private double motorSpeed;
    private FeederSubsystem feederSubsystem;

    public SpinFeederCommand(FeederSubsystem feeder, double speed) {
        motorSpeed = speed;
        feederSubsystem = feeder;
        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        feederSubsystem.spinMotor(motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.spinMotor(0);
    }

}
