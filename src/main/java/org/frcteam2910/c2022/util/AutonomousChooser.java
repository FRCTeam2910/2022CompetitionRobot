package org.frcteam2910.c2022.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frcteam2910.c2022.RobotContainer;
import org.frcteam2910.c2022.commands.*;
import org.frcteam2910.common.control.Path;
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
        autonomousModeChooser.addOption("4 Ball (Orange)", AutonomousMode.FOUR_BALL_ORANGE);
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }

    public Command getFenderBlueAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFenderBluePartOne());

        shootAtTarget(command, container, 1.5);
        follow(command, container, trajectories.getFenderBluePartOne());

        return command;
    }

    public Command getFenderRedAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFenderRedPartOne());

        shootAtTarget(command, container, 1.5);
        follow(command, container, trajectories.getFenderRedPartOne());

        return command;
    }

    public Command get2BallWhiteAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTwoBallWhitePartOne());

        followAndIntake(command, container, trajectories.getTwoBallWhitePartOne());
        shootAtTarget(command, container, 1.5);

        return command;
    }

    public Command get2BallPurpleAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTwoBallPurplePartOne());

        followAndIntake(command, container, trajectories.getTwoBallPurplePartOne());
        shootAtTarget(command, container, 1.5);

        return command;
    }

    public Command get4BallOrangeAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFourBallOrangeAutoPartOne());

        followAndIntake(command, container, trajectories.getFourBallOrangeAutoPartOne());
        shootAtTarget(command, container, 1.5);
        followAndIntake(command, container, trajectories.getFourBallOrangeAutoPartTwo());
        follow(command, container, trajectories.getFourBallOrangeAutoPartThree());
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

    public void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        Path.State start = trajectory.getPath().calculate(0.0);
        command.addCommands(new InstantCommand(() -> container.getDrivetrain().setPose(new Pose2d(start.getPosition().x,
                start.getPosition().y, new Rotation2d(start.getRotation().toRadians())))));
    }

    private enum AutonomousMode {
        FENDER_RED, FENDER_BLUE, TWO_BALL_WHITE, TWO_BALL_PURPLE, FOUR_BALL_ORANGE
    }
}
