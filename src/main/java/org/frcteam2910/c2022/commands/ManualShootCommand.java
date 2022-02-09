package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.FeederSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class ManualShootCommand extends CommandBase {
    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;

    public ManualShootCommand(FeederSubsystem feeder, ShooterSubsystem shooter){
        this.feeder = feeder;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        feeder.setFeederSpeed(0.5);
        shooter.setVoltage(6);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setFeederSpeed(0.0);
        shooter.setVoltage(0.0);
    }
}
