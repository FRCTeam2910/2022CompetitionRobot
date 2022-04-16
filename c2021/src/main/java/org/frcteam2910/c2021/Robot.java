package org.frcteam2910.c2021;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private final RobotContainer container = new RobotContainer();

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        container.getVisual().update();
    }
}
