package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;

public class FeederIntakeWhenNotFullCommand extends CommandBase {
    private FeederSubsystem feederSubsystem;
    private double speed;

    public FeederIntakeWhenNotFullCommand(FeederSubsystem feederSubsystem, double speed) {
        this.feederSubsystem = feederSubsystem;
        this.speed = speed;

        addRequirements(feederSubsystem);
    }

    @Override
    public void execute() {
        if (feederSubsystem.shouldAdvance()) {
            feederSubsystem.spinMotor(speed);
        } else {
            feederSubsystem.spinMotor(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.spinMotor(0.0);
    }
}
