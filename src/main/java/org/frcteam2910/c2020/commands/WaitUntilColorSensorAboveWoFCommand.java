package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.WheelOfFortuneSubsystem;

public class WaitUntilColorSensorAboveWoFCommand extends CommandBase {
    private WheelOfFortuneSubsystem spinner;

    public WaitUntilColorSensorAboveWoFCommand(WheelOfFortuneSubsystem wheelOfFortuneSpinner) {
        spinner = wheelOfFortuneSpinner;
        addRequirements(spinner);
    }

    @Override
    public boolean isFinished() {
        return spinner.isCloseToColorSensor();
    }
}
