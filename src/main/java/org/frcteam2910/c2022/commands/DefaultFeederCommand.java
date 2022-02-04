package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.FeederSubsystem;

public class DefaultFeederCommand extends CommandBase {
    private final FeederSubsystem feeder;

    public DefaultFeederCommand(FeederSubsystem feeder){
        this.feeder = feeder;
        addRequirements(feeder);
    }

    @Override
    public void execute() {
        if(!feeder.getFullSensor() & feeder.getEntrySensor()){
            feeder.setFeederSpeed(0.5);
        } else {
            feeder.setFeederSpeed(0.0);
        }
    }
}
