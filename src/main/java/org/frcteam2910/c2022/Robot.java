package org.frcteam2910.c2022;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frcteam2910.c2022.commands.CharacterizeDrivetrainCommand;
import org.frcteam2910.c2022.commands.ZeroHoodCommand;
import org.frcteam2910.c2022.util.DriverReadout;

public class Robot extends TimedRobot {
    private final RobotContainer robotContainer = new RobotContainer();

    private final CharacterizeDrivetrainCommand characterizeCommand = new CharacterizeDrivetrainCommand(
            robotContainer.getDrivetrain());

    @SuppressWarnings("unused")
    private final DriverReadout driverReadout = new DriverReadout(robotContainer);

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        robotContainer.getShooter().setHoodBrakeMode(false);
    }

    @Override
    public void disabledExit() {
        robotContainer.getShooter().setHoodBrakeMode(false);
    }

    @Override
    public void teleopInit() {
        if (!robotContainer.getShooter().isHoodZeroed()) {
            new ZeroHoodCommand(robotContainer.getShooter(), false).schedule();
        }
        robotContainer.getDrivetrain().setPose(new Pose2d());
    }

    @Override
    public void autonomousInit() {
        robotContainer.getAutonomousChooser().getCommand(robotContainer).schedule();
        // characterizeCommand.schedule();
    }
}
