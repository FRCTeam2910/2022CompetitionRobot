package org.frcteam2910.c2022;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import org.frcteam2910.c2022.commands.*;
import org.frcteam2910.c2022.subsystems.*;
import org.frcteam2910.c2022.util.AutonomousChooser;
import org.frcteam2910.c2022.util.AutonomousTrajectories;
import org.frcteam2910.c2022.util.DriverReadout;
import org.frcteam2910.visionlib.TargetingLookupEntry;
import org.frcteam2910.visionlib.TargetingSolver;
import org.littletonrobotics.oi.OverrideOI;
import org.littletonrobotics.subsystems.Vision;
import org.littletonrobotics.subsystems.VisionIOLimelight;

public class RobotContainer {
    private final Superstructure superstructure = new Superstructure();

    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final FeederSubsystem feeder = new FeederSubsystem();
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    private final Vision vision = new Vision(new VisionIOLimelight(), drivetrain);

    private final TargetingSolver targetingSolver = new TargetingSolver(new Translation2d());

    private final AutonomousChooser autonomousChooser = new AutonomousChooser(
            new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS));

    private final XboxController controller = new XboxController(Constants.CONTROLLER_PORT);

    private final DriverReadout driverReadout = new DriverReadout(this);

    public RobotContainer() {
        SmartDashboard.putBoolean("VisionTuningMode", false);
        vision.setSuppliers(() -> {
            boolean tuningMode = SmartDashboard.getBoolean("VisionTuningMode", false);
            return tuningMode ? OverrideOI.VisionLedMode.ALWAYS_ON : OverrideOI.VisionLedMode.AUTO;
        });

        targetingSolver.addEntry(1.0,
                new TargetingLookupEntry(Math.toRadians(7.0), Units.rotationsPerMinuteToRadiansPerSecond(1725), 1.0));
        targetingSolver.addEntry(1.5,
                new TargetingLookupEntry(Math.toRadians(9.5), Units.rotationsPerMinuteToRadiansPerSecond(1725), 1.1));
        targetingSolver.addEntry(2.0,
                new TargetingLookupEntry(Math.toRadians(11.5), Units.rotationsPerMinuteToRadiansPerSecond(1750), 1.2));
        targetingSolver.addEntry(2.5,
                new TargetingLookupEntry(Math.toRadians(13.5), Units.rotationsPerMinuteToRadiansPerSecond(1825), 1.3));
        targetingSolver.addEntry(3.0,
                new TargetingLookupEntry(Math.toRadians(16.0), Units.rotationsPerMinuteToRadiansPerSecond(1900), 1.4));
        targetingSolver.addEntry(3.5,
                new TargetingLookupEntry(Math.toRadians(18.5), Units.rotationsPerMinuteToRadiansPerSecond(2000), 1.5));
        targetingSolver.addEntry(4.0,
                new TargetingLookupEntry(Math.toRadians(21.5), Units.rotationsPerMinuteToRadiansPerSecond(2100), 1.6));
        targetingSolver.addEntry(4.5,
                new TargetingLookupEntry(Math.toRadians(24.0), Units.rotationsPerMinuteToRadiansPerSecond(2175), 1.7));
        targetingSolver.addEntry(5.0,
                new TargetingLookupEntry(Math.toRadians(25.5), Units.rotationsPerMinuteToRadiansPerSecond(2275), 1.8));
        targetingSolver.addEntry(5.5,
                new TargetingLookupEntry(Math.toRadians(27.5), Units.rotationsPerMinuteToRadiansPerSecond(2400), 1.9));
        targetingSolver.addEntry(6.0,
                new TargetingLookupEntry(Math.toRadians(30.0), Units.rotationsPerMinuteToRadiansPerSecond(2475), 2.0));

        CommandScheduler.getInstance().registerSubsystem(climber);
        CommandScheduler.getInstance().registerSubsystem(shooter);
        CommandScheduler.getInstance().registerSubsystem(intake);
        CommandScheduler.getInstance().registerSubsystem(feeder);
        CommandScheduler.getInstance().registerSubsystem(vision);
        CommandScheduler.getInstance().registerSubsystem(drivetrain);

        shooter.setDefaultCommand(new DefaultShooterCommand(shooter));
        intake.setDefaultCommand(new DefaultIntakeCommand(intake));
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, this::getForwardInput, this::getStrafeInput,
                this::getRotationInput));
        feeder.setDefaultCommand(new DefaultFeederCommand(feeder));
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

    public Vision getVision() {
        return vision;
    }

    public XboxController getController() {
        return controller;
    }

    public void configureButtonBindings() {
        new Button(controller::getLeftBumper).whileHeld(new SimpleIntakeCommand(intake, feeder, controller));
        new Button(() -> controller.getRightTriggerAxis() > 0.5).whileHeld(new FenderShootCommand(feeder, shooter));
        new Button(controller::getYButton).whenPressed(new ZeroClimberCommand(climber));
        new Button(controller::getXButton).whenPressed(new ZeroHoodCommand(shooter, true));
        new Button(controller::getAButton).whileHeld(new ManualFeedToShooterCommand(feeder));
        new Button(() -> controller.getLeftTriggerAxis() > 0.5).whenPressed(new ResetFeederCommand(feeder, intake));
        new Button(controller::getRightBumper).whileHeld(new TargetCommand(drivetrain, shooter, vision, targetingSolver,
                driverReadout, this::getForwardInput, this::getStrafeInput).alongWith(
                        new WaitCommand(0.1).andThen(new ShootWhenReadyCommand(drivetrain, feeder, shooter))));
        new Button(() -> controller.getPOV() == 0).whenPressed(new ConditionalCommand(
                new ClimberToPointCommand(climber, ClimberSubsystem.MID_RUNG_HEIGHT),
                new ClimberToPointCommand(climber, ClimberSubsystem.MAX_HEIGHT), () -> climber
                        .getCurrentHeight() > (ClimberSubsystem.MAX_HEIGHT + ClimberSubsystem.MID_RUNG_HEIGHT) / 2.0));
        new Button(() -> controller.getPOV() == 180)
                .whenPressed(new ClimberToPointCommand(climber, ClimberSubsystem.MIN_HEIGHT));
        new Button(controller::getBackButton).whenPressed(drivetrain::zeroRotation);
        new Button(controller::getStartButton).whenPressed(new AutoClimbCommand(climber, shooter));
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
        // // //manual flywheel adjustment - 90: right, 270: left
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

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    public TargetingSolver getTargetingSolver() {
        return targetingSolver;
    }

    public DriverReadout getDriverReadout() {
        return driverReadout;
    }
}
