package org.frcteam2910.c2022.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frcteam2910.c2022.RobotContainer;
import org.frcteam2910.c2022.commands.*;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.PathSegment;
import org.frcteam2910.common.control.Trajectory;

import java.util.Map;


public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(AutonomousTrajectories trajectories){
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("1 Ball Auto", AutonomousMode.ONE_BALL);
        autonomousModeChooser.addOption("3 Ball Auto", AutonomousMode.THREE_BALL);
        autonomousModeChooser.addOption("4 Ball Auto", AutonomousMode.FOUR_BALL);
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }
    public Command get1BallAuto(RobotContainer container){
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0))); // TODO: Update Code For Beginning Position

        shootAtTarget(command, container, 1.5);
        follow(command, container, trajectories.getOneBallAutoPartOne());

        return command;
    }

    public Command get3BallAuto(RobotContainer container){
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0))); // TODO: Update Code For Beginning Position

        followAndIntake(command, container, trajectories.getThreeBallAutoPartOne());
        follow(command, container, trajectories.getThreeBallAutoPartTwo());
        shootAtTarget(command, container, 1.5);
        followAndIntake(command, container, trajectories.getThreeBallAutoPartThree());
        shootAtTarget(command, container, 1.5);

        return command;
    }

    public Command get4BallAuto(RobotContainer container){
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0))); // TODO: Update Code For Beginning Position

        followAndIntake(command, container, trajectories.getFourBallAutoPartOne());
        follow(command, container, trajectories.getFourBallAutoPartTwo());
        shootAtTarget(command, container, 1.5);
        followAndIntake(command, container, trajectories.getFourBallAutoPartThree());
        follow(command, container, trajectories.getFourBallAutoPartFour());
        shootAtTarget(command, container, 1.5);

        return command;
    }

    public Command getCommand(RobotContainer container){
        switch(autonomousModeChooser.getSelected()){
            case ONE_BALL:
                return get1BallAuto(container);
            case THREE_BALL:
                return get3BallAuto(container);
            case FOUR_BALL:
                return get4BallAuto(container);
        }
        return get4BallAuto(container);
    }

    private void shootAtTarget(SequentialCommandGroup command, RobotContainer container, double timeToWait) {
        command.addCommands(
                new TargetWithShooterCommand(container.getShooter(), container.getVision())
                        .alongWith(new AlignRobotToShootCommand(container.getDrivetrain(), container.getVision()))
                        .alongWith(
                                new WaitCommand(0.1).andThen(new ShootWhenReadyCommand(container.getFeeder(), container.getShooter(), container.getVision())))
                        .alongWith(new TargetWithShooterCommand(container.getShooter(), container.getVision()))
                        .withTimeout(timeToWait));
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrain(), trajectory));
    }

    private void followAndIntake(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(
                new FollowTrajectoryCommand(container.getDrivetrain(), trajectory)
                        .deadlineWith(
                                new SimpleIntakeCommand(container.getIntake())));
    }

    public void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Pose2d pose) {
        command.addCommands(new InstantCommand(() -> container.getDrivetrain().setPose(pose)));
    }

    private enum AutonomousMode {
        ONE_BALL,
        THREE_BALL,
        FOUR_BALL
    }
}
