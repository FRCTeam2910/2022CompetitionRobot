package org.frcteam2910.c2022;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import org.frcteam2910.c2022.commands.*;
import org.frcteam2910.c2022.subsystems.*;
import org.frcteam2910.c2022.util.AutonomousChooser;
import org.frcteam2910.c2022.util.AutonomousTrajectories;
import org.frcteam2910.c2022.util.ClimbChooser;

public class RobotContainer {
    private final Superstructure superstructure = new Superstructure();

    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final FeederSubsystem feeder = new FeederSubsystem();
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain);

    private final AutonomousChooser autonomousChooser = new AutonomousChooser(
            new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS));
    private final ClimbChooser climbChooser = new ClimbChooser();

    private final XboxController controller = new XboxController(Constants.CONTROLLER_PORT);

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(climber);
        CommandScheduler.getInstance().registerSubsystem(shooter);
        CommandScheduler.getInstance().registerSubsystem(intake);
        CommandScheduler.getInstance().registerSubsystem(feeder);
        CommandScheduler.getInstance().registerSubsystem(drivetrain);
        CommandScheduler.getInstance().registerSubsystem(vision);

        shooter.setDefaultCommand(new DefaultShooterCommand(shooter));
        intake.setDefaultCommand(new DefaultIntakeCommand(intake));
        feeder.setDefaultCommand(new DefaultFeederCommand(feeder));
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, this::getForwardInput, this::getStrafeInput,
                this::getRotationInput));

        configureButtonBindings();
    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }

    public ShooterSubsystem getShooter() {
        return shooter;
    }

    public IntakeSubsystem getIntake() {
        return intake;
    }

    public ClimberSubsystem getClimber() {
        return climber;
    }

    public FeederSubsystem getFeeder() {
        return feeder;
    }

    public VisionSubsystem getVision() {
        return vision;
    }

    public XboxController getController() {
        return controller;
    }

    public void configureButtonBindings() {
        new Button(controller::getRightBumper).whileHeld(new TargetWithShooterCommand(shooter, vision)
                .alongWith(
                        new AlignRobotToShootCommand(drivetrain, vision, this::getForwardInput, this::getStrafeInput))
                .alongWith(new WaitCommand(0.1).andThen(new ShootWhenReadyCommand(feeder, shooter, vision))));
        new Button(controller::getLeftBumper).whileHeld(new SimpleIntakeCommand(intake, feeder, controller));

        new Button(() -> controller.getRightTriggerAxis() > 0.5).whileHeld(new FenderShootCommand(feeder, shooter));
        new Button(() -> controller.getLeftTriggerAxis() > 0.5).whenPressed(new ResetFeederCommand(feeder, intake));

        new Button(controller::getYButton).whenPressed(new ZeroClimberCommand(climber));
        new Button(controller::getXButton).whenPressed(new ZeroHoodCommand(shooter, true));
        new Button(controller::getAButton).whileHeld(new ManualFeedToShooterCommand(feeder));

        new Button(() -> controller.getPOV() == 0).whenPressed(new ConditionalCommand(
                new ClimberToPointCommand(climber, ClimberSubsystem.MID_RUNG_HEIGHT, false, true),
                new ClimberToPointCommand(climber, ClimberSubsystem.MAX_HEIGHT, false, true), () -> climber
                        .getCurrentHeight() > (ClimberSubsystem.MAX_HEIGHT + ClimberSubsystem.MID_RUNG_HEIGHT) / 2.0));
        new Button(() -> controller.getPOV() == 180)
                .whenPressed(new ClimberToPointCommand(climber, ClimberSubsystem.MIN_HEIGHT, false, true));

        new Button(controller::getBackButton).whenPressed(drivetrain::zeroRotation);
        new Button(controller::getStartButton).whenPressed(
                new AutoClimbCommand(climber, shooter, () -> climbChooser.getClimbChooser().getSelected()));

        // // manual hood adjustment - 0: up, 180: down
        // new Button(() -> controller.getPOV() == 180.0).whenPressed(() ->
        // shooter.setHoodTargetPosition(
        // shooter.getHoodTargetPosition() - Constants.HOOD_MANUAL_ADJUST_INTERVAL)
        // );
        //
        // new Button(() -> controller.getPOV() == 0.0).whenPressed(() ->
        // shooter.setHoodTargetPosition(
        // shooter.getHoodTargetPosition() + Constants.HOOD_MANUAL_ADJUST_INTERVAL)
        // );
        //
        // //manual flywheel adjustment - 90: right, 270: left
        // new Button(() -> controller.getPOV() == 90.0).whenPressed(() ->
        // shooter.setTargetFlywheelSpeed(
        // shooter.getTargetFlywheelSpeed() + Constants.FLYWHEEL_MANUAL_ADJUST_INTERVAL)
        // );
        //
        // new Button(() -> controller.getPOV() == 270.0).whenPressed(() ->
        // shooter.setTargetFlywheelSpeed(
        // shooter.getTargetFlywheelSpeed() - Constants.FLYWHEEL_MANUAL_ADJUST_INTERVAL)
        // );
    }

    private static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance)
            return 0.0;

        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }

    private double getForwardInput() {
        return -square(deadband(controller.getLeftY(), 0.1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    }

    private double getStrafeInput() {
        return -square(deadband(controller.getLeftX(), 0.1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    }

    private double getRotationInput() {
        return -square(deadband(controller.getRightX(), 0.1))
                * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    public AutonomousChooser getAutonomousChooser() {
        return autonomousChooser;
    }

    public ClimbChooser getClimbChooser() {
        return climbChooser;
    }

    public Superstructure getSuperstructure() {
        return superstructure;
    }
}
