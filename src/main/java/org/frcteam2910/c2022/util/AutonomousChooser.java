package org.frcteam2910.c2022.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import org.frcteam2910.c2022.RobotContainer;
import org.frcteam2910.c2022.commands.*;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.Trajectory;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("Five Ball (Orange)", AutonomousMode.FIVE_BALL_ORANGE);
        autonomousModeChooser.addOption("Test Auto", AutonomousMode.TEST_AUTO);
        autonomousModeChooser.addOption("Fender (Red)", AutonomousMode.FENDER_RED);
        autonomousModeChooser.addOption("Fender (Blue)", AutonomousMode.FENDER_BLUE);
        autonomousModeChooser.addOption("Two Ball (Green)", AutonomousMode.TWO_BALL_GREEN);
        autonomousModeChooser.addOption("Two Ball (Purple)", AutonomousMode.TWO_BALL_PURPLE);
        autonomousModeChooser.addOption("Three Ball (Orange)", AutonomousMode.THREE_BALL_ORANGE);
        autonomousModeChooser.addOption("Five Ball (Orange)", AutonomousMode.FIVE_BALL_ORANGE);
        autonomousModeChooser.addOption("Five Ball Defensive (Orange)", AutonomousMode.FIVE_BALL_DEFENSIVE);
        autonomousModeChooser.addOption("Six Ball (Orange)", AutonomousMode.SIX_BALL_ORANGE);
        autonomousModeChooser.addOption("Two Ball Defensive (Green)", AutonomousMode.TWO_BALL_DEFENSIVE);
        autonomousModeChooser.addOption("Fender Double Preload (Red)", AutonomousMode.FENDER_DOUBLE_PRELOAD_RED);
        autonomousModeChooser.addOption("Two Ball Double Preload (Green)",
                AutonomousMode.TWO_BALL_DOUBLE_PRELOAD_GREEN);
        autonomousModeChooser.addOption("Three Ball Double Preload (Orange)",
                AutonomousMode.THREE_BALL_DOUBLE_PRELOAD_ORANGE);
        autonomousModeChooser.addOption("Five Ball Double Preload (Orange)",
                AutonomousMode.FIVE_BALL_DOUBLE_PRELOAD_ORANGE);
    }

    public SendableChooser<AutonomousMode> getModeChooser() {
        return autonomousModeChooser;
    }

    public Command getTestAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTestAutoPartOne());

        command.addCommands(follow(container, trajectories.getTestAutoPartOne()));

        return command;
    }

    public Command getFenderBlueAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFenderBluePartOne());
        command.addCommands(new ZeroHoodCommand(container.getShooter(), true));

        command.addCommands(new FenderShootCommand(container.getFeeder(), container.getShooter()).withTimeout(1.5));
        command.addCommands(follow(container, trajectories.getFenderBluePartOne()));

        return command;
    }

    public Command getFenderRedAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFenderRedPartOne());

        command.addCommands(new ZeroHoodCommand(container.getShooter(), true));

        command.addCommands(new FenderShootCommand(container.getFeeder(), container.getShooter()).withTimeout(2.5));
        command.addCommands(follow(container, trajectories.getFenderRedPartOne()));

        return command;
    }

    public Command get2BallPurpleAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTwoBallPurplePartOne());

        command.addCommands(followAndIntake(container, trajectories.getTwoBallPurplePartOne())
                .alongWith(new ZeroHoodCommand(container.getShooter(), true)));
        shootAtTarget(command, container, 1.5);

        return command;
    }

    public Command get2BallGreenAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTwoBallGreenPartOne());

        command.addCommands(followAndIntake(container, trajectories.getTwoBallGreenPartOne())
                .alongWith(new ZeroHoodCommand(container.getShooter(), true)));
        shootAtTarget(command, container, 2.0);

        return command;
    }

    public Command get3BallOrangeAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getThreeBallOrangePartOne());

        // Grab the second ball and move to the shooting position
        // Zero the hood along the way
        command.addCommands(followAndIntake(container, trajectories.getThreeBallOrangePartOne())
                .andThen(follow(container, trajectories.getThreeBallOrangePartTwo()))
                .alongWith(new ZeroHoodCommand(container.getShooter(), true, true)));

        // Shoot the 1st and 2nd balls
        shootAtTarget(command, container, 1.5);

        // Grab the 3rd ball
        command.addCommands(followAndIntake(container, trajectories.getThreeBallOrangePartThree()));

        // Shoot the 3rd ball
        shootAtTarget(command, container, 1.5);

        return command;
    }

    public Command get5BallOrangeAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // First run the three ball
        command.addCommands(get3BallOrangeAuto(container));

        // Grab the 4th ball and wait for the 5th
        command.addCommands(followAndIntake(container, trajectories.getFiveBallOrangePartOne(),
                () -> container.getFeeder().isFull(), 0.0));
        command.addCommands(followAndIntake(container, trajectories.getFiveBallOrangePartTwo(),
                () -> container.getFeeder().isFull(), 2.0));

        // Go to shooting location
        command.addCommands(follow(container, trajectories.getFiveBallOrangePartThree()));

        // Shoot the 4th and 5th balls
        shootAtTarget(command, container, 1.5);

        return command;
    }

    public Command get6BallOrangeAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getSixBallOrangePartOne());

        // Grab the second ball and move to the shooting position
        // Zero the hood along the way
        command.addCommands(followAndIntake(container, trajectories.getSixBallOrangePartOne())
                .andThen(follow(container, trajectories.getSixBallOrangePartTwo()))
                .alongWith(new ZeroHoodCommand(container.getShooter(), true, true)));

        // Shoot the 1st and 2nd balls
        shootAtTarget(command, container, 1.5);

        // Grab the 3rd ball
        command.addCommands(followAndIntake(container, trajectories.getSixBallOrangePartThree()));

        // Shoot the 3rd ball
        shootAtTarget(command, container, 1.0);

        // Grab the 4th ball and wait for the 5th
        command.addCommands(followAndIntake(container, trajectories.getSixBallOrangePartFour(),
                () -> container.getFeeder().isFull(), 1.0));

        // Retract Intake
        command.addCommands(new InstantCommand(() -> container.getIntake().setExtended(false)));

        // Pass through the hangar to the 6th
        command.addCommands(follow(container, trajectories.getSixBallOrangePartFive()));
        command.addCommands(new InstantCommand(() -> container.getIntake().setExtended(true)));

        // Shoot 4th and 5th
        shootAtTarget(command, container, 1.5);

        // Intake 6th
        command.addCommands(followAndIntake(container, trajectories.getSixBallOrangePartSix()));

        // Shoot 6th
        shootAtTarget(command, container, 1.5);

        return command;
    }

    public Command get2BallDefensiveAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // Run 2 ball white
        command.addCommands(get2BallGreenAuto(container));

        // Pick up both opponent ball
        command.addCommands(followAndIntake(container, trajectories.getTwoBallDefensivePartOne()));

        // Move up to hub
        command.addCommands(new InstantCommand(() -> container.getIntake().setExtended(false)));
        command.addCommands(follow(container, trajectories.getTwoBallDefensivePartTwo()));

        // Reverse feed ball
        command.addCommands(new ResetFeederCommand(container.getFeeder(), container.getIntake()));

        return command;
    }

    public Command getFenderTwoPreloadRedAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFenderRedPartOne());

        command.addCommands(
                new SimpleIntakeCommand(container.getIntake(), container.getFeeder(), container.getController())
                        .withTimeout(1.0).alongWith(new ZeroHoodCommand(container.getShooter(), true, true)));

        command.addCommands(new FenderShootCommand(container.getFeeder(), container.getShooter()).withTimeout(2.5));
        command.addCommands(follow(container, trajectories.getFenderRedPartOne()));

        return command;
    }

    public Command get2BallTwoPreloadGreenAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTwoBallGreenPartOne());

        // Grab the 2nd preloaded ball
        // Zero the hood along the way
        command.addCommands(
                new SimpleIntakeCommand(container.getIntake(), container.getFeeder(), container.getController())
                        .withTimeout(1.0).alongWith(new ZeroHoodCommand(container.getShooter(), true, true)));

        // Shoot preloaded balls
        shootAtTarget(command, container, 1.5);

        command.addCommands(followAndIntake(container, trajectories.getTwoBallGreenPartOne()));
        shootAtTarget(command, container, 2.0);

        return command;
    }

    public Command get3BallTwoPreloadOrangeAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getThreeBallTwoPreloadOrangePartOne());

        // Grab the 2nd preloaded ball
        // Zero the hood along the way
        command.addCommands(
                new SimpleIntakeCommand(container.getIntake(), container.getFeeder(), container.getController())
                        .withTimeout(1.0).alongWith(new ZeroHoodCommand(container.getShooter(), true, true)));

        // Shoot preloaded balls
        shootAtTarget(command, container, 1.5);

        // Grab the 2nd ball
        command.addCommands(followAndIntake(container, trajectories.getThreeBallTwoPreloadOrangePartOne()));

        // Grab the 3rd ball
        command.addCommands(followAndIntake(container, trajectories.getThreeBallTwoPreloadOrangePartTwo()));

        // Shoot the 2nd and 3rd ball
        shootAtTarget(command, container, 1.5);

        return command;
    }

    public Command get5BallTwoPreloadOrangeAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // First run the three ball
        command.addCommands(get3BallTwoPreloadOrangeAuto(container));

        // Grab the 4th ball and wait for the 5th
        command.addCommands(followAndIntake(container, trajectories.getFiveBallTwoPreloadOrangePartOne(),
                () -> container.getFeeder().isFull(), 0.0));
        command.addCommands(followAndIntake(container, trajectories.getFiveBallTwoPreloadOrangePartTwo(),
                () -> container.getFeeder().isFull(), 2.0));

        // Go to shooting location
        command.addCommands(follow(container, trajectories.getFiveBallTwoPreloadOrangePartThree()));

        // Shoot the 4th and 5th balls
        shootAtTarget(command, container, 1.5);

        return command;
    }

    public Command get5BallDefensiveAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // First run the three ball
        command.addCommands(get3BallOrangeAuto(container));

        // Grab the 4th ball and wait for the 5th
        command.addCommands(followAndIntake(container, trajectories.getFiveBallOrangePartOne(),
                () -> container.getFeeder().isFull(), 1.0));
        command.addCommands(followAndIntake(container, trajectories.getFiveBallOrangePartTwo(),
                () -> container.getFeeder().isFull(), 1.0));

        // Retract Intake
        command.addCommands(new InstantCommand(() -> container.getIntake().setExtended(false)));

        // Go to shooting location
        command.addCommands(follow(container, trajectories.getFiveBallDefensivePartThree()));

        // Shoot the 4th and 5th balls
        shootAtTarget(command, container, 1.5);

        return command;
    }

    private void shootAtTarget(SequentialCommandGroup command, RobotContainer container, double timeToWait) {
        command.addCommands(new TargetWithShooterCommand(container.getShooter(), container.getVision())
                .alongWith(new AlignRobotToShootCommand(container.getDrivetrain(), container.getVision()))
                .alongWith(new WaitCommand(0.1).andThen(new ShootWhenReadyCommand(container.getFeeder(),
                        container.getShooter(), container.getVision())))
                .withTimeout(timeToWait));
    }

    private Command follow(RobotContainer container, Trajectory trajectory) {
        return new FollowTrajectoryCommand(container.getDrivetrain(), trajectory);
    }

    private Command followAndIntake(RobotContainer container, Trajectory trajectory) {
        return followAndIntake(container, trajectory, () -> true, 0.0);
    }

    private Command followAndIntake(RobotContainer container, Trajectory trajectory, BooleanSupplier condition,
            double conditionTimeout) {
        return new FollowTrajectoryCommand(container.getDrivetrain(), trajectory)
                .andThen(new WaitUntilCommand(condition).withTimeout(conditionTimeout))
                .deadlineWith(new SimpleIntakeCommand(container.getIntake(), container.getFeeder(),
                        container.getController()), new DefaultFeederCommand(container.getFeeder()));
    }

    public void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        Path.State start = trajectory.getPath().calculate(0.0);
        command.addCommands(new InstantCommand(() -> container.getDrivetrain().setPose(new Pose2d(start.getPosition().x,
                start.getPosition().y, new Rotation2d(start.getRotation().toRadians())))));
    }

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case TEST_AUTO :
                return getTestAuto(container);
            case FENDER_RED :
                return getFenderRedAuto(container);
            case FENDER_BLUE :
                return getFenderBlueAuto(container);
            case TWO_BALL_PURPLE :
                return get2BallPurpleAuto(container);
            case TWO_BALL_GREEN :
                return get2BallGreenAuto(container);
            case THREE_BALL_ORANGE :
                return get3BallOrangeAuto(container);
            case FIVE_BALL_ORANGE :
                return get5BallOrangeAuto(container);
            case FIVE_BALL_DEFENSIVE :
                return get5BallDefensiveAuto(container);
            case SIX_BALL_ORANGE :
                return get6BallOrangeAuto(container);
            case TWO_BALL_DEFENSIVE :
                return get2BallDefensiveAuto(container);
            case FENDER_DOUBLE_PRELOAD_RED :
                return getFenderTwoPreloadRedAuto(container);
            case TWO_BALL_DOUBLE_PRELOAD_GREEN :
                return get2BallTwoPreloadGreenAuto(container);
            case THREE_BALL_DOUBLE_PRELOAD_ORANGE :
                return get3BallTwoPreloadOrangeAuto(container);
            case FIVE_BALL_DOUBLE_PRELOAD_ORANGE :
                return get5BallTwoPreloadOrangeAuto(container);
        }
        return new InstantCommand();
    }

    private enum AutonomousMode {
        TEST_AUTO, FENDER_RED, FENDER_BLUE, TWO_BALL_GREEN, TWO_BALL_PURPLE, THREE_BALL_ORANGE, FIVE_BALL_ORANGE, FIVE_BALL_DEFENSIVE, SIX_BALL_ORANGE, TWO_BALL_DEFENSIVE, FENDER_DOUBLE_PRELOAD_RED, TWO_BALL_DOUBLE_PRELOAD_GREEN, THREE_BALL_DOUBLE_PRELOAD_ORANGE, FIVE_BALL_DOUBLE_PRELOAD_ORANGE
    }
}