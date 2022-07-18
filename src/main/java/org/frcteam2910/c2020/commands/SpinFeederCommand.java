package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;
import org.frcteam2910.c2020.subsystems.IntakeSubsystem;

public class SpinFeederCommand extends CommandBase {

    private double motorSpeed;
    private FeederSubsystem feederSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public SpinFeederCommand(FeederSubsystem feeder, IntakeSubsystem intakeSubsystem, double speed) {
        motorSpeed = speed;
        feederSubsystem = feeder;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        feederSubsystem.spinMotor(motorSpeed);
        intakeSubsystem.setMotorOutput(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.spinMotor(0);
        intakeSubsystem.setMotorOutput(0.0);
    }

}
