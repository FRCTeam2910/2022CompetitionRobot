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
import org.frcteam2910.common.robot.Utilities;

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

    private final XboxController controller = new XboxController(Constants.CONTROLLER_PORT);

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(climber);
        CommandScheduler.getInstance().registerSubsystem(shooter);
        CommandScheduler.getInstance().registerSubsystem(intake);
        CommandScheduler.getInstance().registerSubsystem(feeder);
        CommandScheduler.getInstance().registerSubsystem(vision);
        CommandScheduler.getInstance().registerSubsystem(drivetrain);

        shooter.setDefaultCommand(new DefaultShooterCommand(shooter));
        intake.setDefaultCommand(new DefaultIntakeCommand(intake));
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, this::getTranslationXInput,
                this::getTranslationYInput, () -> -Utilities.deadband(controller.getRightX(), 0.1)
                        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
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

    public VisionSubsystem getVision() {
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
        new Button(controller::getRightBumper).whileHeld(new TargetWithShooterCommand(shooter, drivetrain, vision)
                .alongWith(new AlignRobotToShootCommand(drivetrain, vision, this::getTranslationXInput,
                        this::getTranslationYInput))
                .alongWith(new WaitCommand(0.1).andThen(new ShootWhenReadyCommand(feeder, shooter, vision))));
        new Button(() -> controller.getPOV() == 0)
                .whenPressed(new ConditionalCommand(new ClimberToPointCommand(climber, 0.75),
                        new ClimberToPointCommand(climber, 1.0), () -> climber.getCurrentPosition() > 0.9));
        new Button(() -> controller.getPOV() == 180).whenPressed(new ClimberToPointCommand(climber, 0.0));
        new Button(controller::getBackButton).whenPressed(drivetrain::zeroRotation);
        new Button(controller::getStartButton).whenPressed(
                // a to c
                new PrepareHoodTransferCommand(climber, shooter)
                        // c to e
                        .andThen(new TransferBarToHoodCommand(climber, shooter))
                        // c to f
                        .andThen(new TraverseToNextBarCommand(climber, shooter))
                        // f to c
                        .andThen(new PrepareHoodTransferCommand(climber, shooter))
                        // c to e
                        .andThen(new TransferBarToHoodCommand(climber, shooter))
                        // e to f
                        .andThen(new TraverseToNextBarCommand(climber, shooter))
                        // f to c
                        .andThen(new PrepareHoodTransferCommand(climber, shooter))
                        // c to e
                        .andThen(new TransferBarToHoodCommand(climber, shooter)));
        // //manual hood adjustment - 0: up, 180: down
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

    private double getTranslationXInput() {
        return -Utilities.deadband(controller.getLeftY(), 0.1) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    }

    private double getTranslationYInput() {
        return -Utilities.deadband(controller.getLeftX(), 0.1) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    }

    public AutonomousChooser getAutonomousChooser() {
        return autonomousChooser;
    }

    public Superstructure getSuperstructure() {
        return superstructure;
    }
}
