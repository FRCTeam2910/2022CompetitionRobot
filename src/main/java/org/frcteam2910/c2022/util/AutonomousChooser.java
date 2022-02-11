package org.frcteam2910.c2022.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frcteam2910.c2022.RobotContainer;
import org.frcteam2910.c2022.commands.AutoShootCommand;
import org.frcteam2910.c2022.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2022.commands.SimpleIntakeCommand;
import org.frcteam2910.c2022.commands.TargetWithShooterCommand;
import org.frcteam2910.common.control.Trajectory;


public class AutonomousChooser {
    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(){
        autonomousModeChooser.setDefaultOption("1 Ball Auto", AutonomousMode.ONE_BALL);
        autonomousModeChooser.addOption("3 Ball Auto", AutonomousMode.THREE_BALL);
        autonomousModeChooser.addOption("5 Ball Auto", AutonomousMode.FIVE_BALL);
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser(){
        return autonomousModeChooser;
    }
    public Command get1BallAuto(){
        SequentialCommandGroup command = new SequentialCommandGroup();

        //Add code here

        return command;
    }

    public Command get3BallAuto(){
        SequentialCommandGroup command = new SequentialCommandGroup();

        //Add code here

        return command;
    }

    public Command get5BallAuto(){
        SequentialCommandGroup command = new SequentialCommandGroup();

        //Add code here

        return command;
    }

    private void shootAtTarget(SequentialCommandGroup command, RobotContainer container, double timeToWait) {
        command.addCommands(
                new TargetWithShooterCommand(container.getShooter(), container.getVision())
                        .alongWith(new AutoShootCommand(container.getShooter(), container.getFeeder(), container.getDrivetrain(), container.getVision()))
                        .alongWith(
                                new WaitCommand(0.1).andThen(new AutonomousFeedCommand(container.getShooter(), container.getFeeder(), container.getVision())))
                        .withTimeout(timeToWait));
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrain(), trajectory)
                .deadlineWith(new TargetWithShooterCommand(container.getShooter(), container.getVision())));
    }

    private void followAndIntake(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() -> container.getIntake().setExtended(true)));
        command.addCommands(
                new FollowTrajectoryCommand(container.getDrivetrain(), trajectory)
                        .deadlineWith(
                                new SimpleIntakeCommand(container.getIntake()).withTimeout(0.25)
                                        .andThen(
                                                new SimpleIntakeCommand(container.getIntake())
                                                        )));
        command.addCommands(new InstantCommand(() -> container.getIntake().setExtended(false)));
    }


    public void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Pose2d pose){
        command.addCommands(new InstantCommand(() -> container.getDrivetrain().setPose(
                pose)));
    }

    private enum AutonomousMode {
        ONE_BALL,
        THREE_BALL,
        FIVE_BALL
    }
}