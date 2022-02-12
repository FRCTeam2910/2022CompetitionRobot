package org.frcteam2910.c2022;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private final RobotContainer robotContainer = new RobotContainer();

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
