package org.frcteam2910.c2022;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frcteam2910.c2022.commands.ZeroHoodCommand;

public class Robot extends TimedRobot {
    private final RobotContainer robotContainer = new RobotContainer();

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        if (!robotContainer.getShooter().isHoodZeroed()) {
            new ZeroHoodCommand(robotContainer.getShooter(), false).schedule();
        }
    }
}
