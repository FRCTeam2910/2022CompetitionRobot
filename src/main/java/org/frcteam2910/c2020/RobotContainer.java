package org.frcteam2910.c2020;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousChooser;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.DriverReadout;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.DPadButton;
import org.frcteam2910.common.robot.input.XboxController;

public class RobotContainer {
    private static final double HOOD_MANUAL_ADJUST_INTERVAL = Math.toRadians(0.5);
    private static final double FLYWHEEL_MANUAL_ADJUST_INTERVAL = 50.0;

    private final XboxController primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);

    private final Superstructure superstructure = new Superstructure();

    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrainSubsystem);

    private AutonomousTrajectories autonomousTrajectories;
    private final AutonomousChooser autonomousChooser;

    private final DriverReadout driverReadout;

    public RobotContainer() {
        try {
            autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS);
        } catch (IOException e) {
            e.printStackTrace();
        }
        autonomousChooser = new AutonomousChooser(autonomousTrajectories);

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        CommandScheduler.getInstance().registerSubsystem(climberSubsystem);
        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(shooterSubsystem);
        CommandScheduler.getInstance().registerSubsystem(visionSubsystem);
        CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
        CommandScheduler.getInstance().registerSubsystem(feederSubsystem);

        CommandScheduler.getInstance().setDefaultCommand(climberSubsystem, new DefaultClimberCommand(climberSubsystem));
        CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem,
                getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
        // CommandScheduler.getInstance().setDefaultCommand(feederSubsystem, new
        // FeederIntakeWhenNotFullCommand(feederSubsystem, 0.9));

        // CommandScheduler.getInstance().setDefaultCommand(shooterSubsystem, new
        // ManuallyAdjustShooterCommand(shooterSubsystem));
        CommandScheduler.getInstance().setDefaultCommand(shooterSubsystem,
                new DefaultShooterCommand(shooterSubsystem, Constants.FLYWHEEL_IDLE_SPEED, Constants.HOOD_MAX_ANGLE));
        CommandScheduler.getInstance().setDefaultCommand(intakeSubsystem,
                new ExtendBottomIntakeCommand(intakeSubsystem));// Remove this command as it is not needed; make the
                                                                // field true in the intake subsystem
        CommandScheduler.getInstance().setDefaultCommand(intakeSubsystem,
                new DefaultIntakeCommand(intakeSubsystem, feederSubsystem, superstructure));

        // CommandScheduler.getInstance().setDefaultCommand(feederSubsystem,new
        // RetractTopIntakeWhenNoFifthBallCommand(feederSubsystem,intakeSubsystem));

        driverReadout = new DriverReadout(this);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        primaryController.getBackButton().whenPressed(() -> drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO));
        primaryController.getStartButton().whenPressed(new SpinFlywheelCommand(shooterSubsystem, 0.0));

        primaryController.getLeftBumperButton()
                .whileHeld(new SimpleIntakeCommand(intakeSubsystem, feederSubsystem, primaryController, 1.0, 0.9)
                        .alongWith(new DriveWithSetRotationCommand(drivetrainSubsystem, getDriveForwardAxis(),
                                getDriveStrafeAxis(), 0.0).raceWith(
                                        new WaitCommand(0.5).andThen(new WaitUntilTargetFoundCommand(visionSubsystem)))
                                        .andThen(new DriveToLoadingStationCommand(drivetrainSubsystem, visionSubsystem,
                                                driverReadout::getSelectedLoadingBay))));
        primaryController.getLeftBumperButton().whenReleased(
                new RetractIntakeCommand(feederSubsystem, intakeSubsystem, superstructure).withTimeout(1.0));

        primaryController.getLeftTriggerAxis().getButton(0.5).whenPressed(() -> intakeSubsystem.setTopExtended(true));
        primaryController.getLeftTriggerAxis().getButton(0.5)
                .whileHeld(new SimpleIntakeCommand(intakeSubsystem, feederSubsystem, primaryController, 1.0, 0.9));
        primaryController.getLeftTriggerAxis().getButton(0.5).whenReleased(
                new RetractIntakeCommand(feederSubsystem, intakeSubsystem, superstructure).withTimeout(1.0));

        primaryController.getLeftTriggerAxis().getButton(0.5)
                .whileHeld(new SpinFeederCommand(feederSubsystem, intakeSubsystem, -0.5));

        primaryController.getRightTriggerAxis().getButton(0.5)
                .whileHeld(new FeedBallsToShooterCommand(feederSubsystem, shooterSubsystem, intakeSubsystem));

        primaryController.getRightBumperButton()
                .whileHeld(new TargetWithShooterCommand(shooterSubsystem, visionSubsystem, primaryController).alongWith(
                        new VisionRotateToTargetCommand(drivetrainSubsystem, visionSubsystem,
                                () -> getDriveForwardAxis().get(true), () -> getDriveStrafeAxis().get(true)),
                        new AutoFeedCommand(drivetrainSubsystem, feederSubsystem, shooterSubsystem, visionSubsystem,
                                intakeSubsystem)));

        primaryController.getAButton().whenPressed(
                new BasicDriveCommand(drivetrainSubsystem, new Vector2(-0.5, 0.0), 0.0, false).withTimeout(0.3));
        primaryController.getAButton().whileHeld(new WaitCommand(0.6)
                .andThen(new FeedBallsToShooterCommand(feederSubsystem, shooterSubsystem, intakeSubsystem)));

        primaryController.getXButton().whenPressed(new HomeHoodMotorCommand(shooterSubsystem));

        primaryController.getBButton().whenPressed(() -> {
            Command command = CommandScheduler.getInstance().requiring(shooterSubsystem);
            if (command != null) {
                command.cancel();
            }
        });

        // primaryController.getAButton().whileHeld(new
        // ManuallyAdjustShooterCommand(shooterSubsystem).alongWith(new
        // VisionRotateToTargetCommand(drivetrainSubsystem, visionSubsystem, () ->
        // getDriveForwardAxis().get(true), () -> getDriveStrafeAxis().get(true))));

        primaryController.getYButton().whenPressed(new HomeClimberCommand(climberSubsystem, superstructure));

        // Climber movement
        primaryController.getDPadButton(DPadButton.Direction.UP)
                .whileHeld(new MoveClimberCommand(climberSubsystem, shooterSubsystem, 1.0));
        primaryController.getDPadButton(DPadButton.Direction.DOWN)
                .whileHeld(new MoveClimberCommand(climberSubsystem, shooterSubsystem, -1.0));

        primaryController.getDPadButton(DPadButton.Direction.LEFT)
                .whileHeld(new SpinFeederCommand(feederSubsystem, intakeSubsystem, -0.5));

        // Manual hood adjustment
        // primaryController.getDPadButton(DPadButton.Direction.DOWN).whenPressed(
        // () ->
        // shooterSubsystem.setHoodTargetAngle(shooterSubsystem.getHoodTargetAngle().orElse(Constants.HOOD_MAX_ANGLE)
        // + HOOD_MANUAL_ADJUST_INTERVAL)
        // );
        // primaryController.getDPadButton(DPadButton.Direction.UP).whenPressed(
        // () ->
        // shooterSubsystem.setHoodTargetAngle(shooterSubsystem.getHoodTargetAngle().orElse(Constants.HOOD_MAX_ANGLE)
        // - HOOD_MANUAL_ADJUST_INTERVAL)
        // );

        // Manual flywheel adjustment
        // primaryController.getDPadButton(DPadButton.Direction.RIGHT).whenPressed(
        // () ->
        // shooterSubsystem.shootFlywheel(shooterSubsystem.getFlywheelTargetVelocity() +
        // FLYWHEEL_MANUAL_ADJUST_INTERVAL)
        // );
        // primaryController.getDPadButton(DPadButton.Direction.LEFT).whenPressed(
        // () ->
        // shooterSubsystem.shootFlywheel(shooterSubsystem.getFlywheelTargetVelocity() -
        // FLYWHEEL_MANUAL_ADJUST_INTERVAL)
        // );
    }

    public Command getAutonomousCommand() {
        return autonomousChooser.getCommand(this);
    }

    private Axis getDriveForwardAxis() {
        return primaryController.getLeftYAxis();
    }

    private Axis getDriveStrafeAxis() {
        return primaryController.getLeftXAxis();
    }

    private Axis getDriveRotationAxis() {
        return primaryController.getRightXAxis();
    }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return drivetrainSubsystem;
    }

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    public FeederSubsystem getFeederSubsystem() {
        return feederSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    public ClimberSubsystem getClimberSubsystem() {
        return climberSubsystem;
    }

    public XboxController getPrimaryController() {
        return primaryController;
    }

    public AutonomousChooser getAutonomousChooser() {
        return autonomousChooser;
    }
}
