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
import org.frcteam2910.common.control.Trajectory;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("Fender (Blue)", AutonomousMode.FENDER_BLUE);
        autonomousModeChooser.addOption("Fender (Red)", AutonomousMode.FENDER_RED);
        autonomousModeChooser.addOption("Two Ball (White)", AutonomousMode.TWO_BALL_WHITE);
        autonomousModeChooser.addOption("Two Ball (Purple)", AutonomousMode.TWO_BALL_PURPLE);
        autonomousModeChooser.addOption("3 Ball (Orange)", AutonomousMode.THREE_BALL_ORANGE);
        autonomousModeChooser.addOption("4 Ball (Orange)", AutonomousMode.FOUR_BALL_ORANGE);
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }

    public Command getFenderBlueAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // TODO: Update code For starting position
        resetRobotPose(command, container, new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)));

        shootAtTarget(command, container, 1.5);
        follow(command, container, trajectories.getFenderBluePartOne());

        return command;
    }

    public Command getFenderRedAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // TODO: Update Code for beginning position
        resetRobotPose(command, container, new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)));

        shootAtTarget(command, container, 1.5);
        follow(command, container, trajectories.getFenderRedPartOne());

        return command;
    }

    public Command get2BallWhiteAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)));

        followAndIntake(command, container, trajectories.getTwoBallWhitePartOne());
        shootAtTarget(command, container, 1.5);

        return command;
    }

    public Command get2BallPurpleAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)));

        followAndIntake(command, container, trajectories.getTwoBallPurplePartOne());
        shootAtTarget(command, container, 1.5);

        return command;
    }

    public Command get3BallOrangeAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // TODO: Update code For starting position
        resetRobotPose(command, container, new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)));

        followAndIntake(command, container, trajectories.getThreeBallOrangeAutoPartOne());
        follow(command, container, trajectories.getThreeBallOrangeAutoPartTwo());
        shootAtTarget(command, container, 1.5);
        followAndIntake(command, container, trajectories.getThreeBallOrangeAutoPartThree());
        shootAtTarget(command, container, 1.5);

        return command;
    }

    public Command get4BallOrangeAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // TODO: Update code For starting position
        resetRobotPose(command, container, new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)));

        followAndIntake(command, container, trajectories.getFourBallOrangeAutoPartOne());
        follow(command, container, trajectories.getFourBallOrangeAutoPartTwo());
        shootAtTarget(command, container, 1.5);
        followAndIntake(command, container, trajectories.getFourBallOrangeAutoPartThree());
        follow(command, container, trajectories.getFourBallOrangeAutoPartFour());
        shootAtTarget(command, container, 1.5);

        return command;
    }

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case FENDER_BLUE :
                return getFenderBlueAuto(container);
            case FENDER_RED :
                return getFenderRedAuto(container);
            case TWO_BALL_PURPLE :
                return get2BallPurpleAuto(container);
            case TWO_BALL_WHITE :
                return get2BallWhiteAuto(container);
            case THREE_BALL_ORANGE :
                return get3BallOrangeAuto(container);
            case FOUR_BALL_ORANGE :
                return get4BallOrangeAuto(container);
        }
        return get4BallOrangeAuto(container);
    }

    private void shootAtTarget(SequentialCommandGroup command, RobotContainer container, double timeToWait) {
        command.addCommands(
                new TargetWithShooterCommand(container.getShooter(), container.getDrivetrain(), container.getVision())
                        .alongWith(new AlignRobotToShootCommand(container.getDrivetrain(), container.getVision()))
                        .alongWith(new WaitCommand(0.1).andThen(new ShootWhenReadyCommand(container.getFeeder(),
                                container.getShooter(), container.getVision())))
                        .withTimeout(timeToWait));
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrain(), trajectory));
    }

    private void followAndIntake(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrain(), trajectory).deadlineWith(
                new SimpleIntakeCommand(container.getIntake(), container.getFeeder(), container.getController())));
    }

    public void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Pose2d pose) {
        command.addCommands(new InstantCommand(() -> container.getDrivetrain().setPose(pose)));
    }

    private enum AutonomousMode {
        FENDER_RED, FENDER_BLUE, TWO_BALL_WHITE, TWO_BALL_PURPLE, THREE_BALL_ORANGE, FOUR_BALL_ORANGE
    }
}
