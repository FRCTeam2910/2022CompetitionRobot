package org.frcteam2910.c2022;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
import org.frcteam2910.c2022.commands.*;
import org.frcteam2910.c2022.subsystems.*;

public class RobotContainer {

    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final FeederSubsystem feeder = new FeederSubsystem();
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain);

    private final XboxController controller = new XboxController(Constants.CONTROLLER_PORT);

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(climber);
        CommandScheduler.getInstance().registerSubsystem(shooter);
        CommandScheduler.getInstance().registerSubsystem(intake);
        CommandScheduler.getInstance().registerSubsystem(feeder);
        CommandScheduler.getInstance().registerSubsystem(vision);
        CommandScheduler.getInstance().registerSubsystem(drivetrain);

        shooter.setDefaultCommand(new DefaultShooterCommand(shooter,
                () -> controller.getRawAxis(0) * 12,
                () -> controller.getRawAxis(1) * 12));
        intake.setDefaultCommand(new DefaultIntakeCommand(intake));
        climber.setDefaultCommand(new DefaultClimberCommand(climber,
                () -> controller.getRawAxis(2)
        ));
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain,
            () -> controller.getRawAxis(3) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> controller.getRawAxis(4) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> controller.getRawAxis(5) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));
        feeder.setDefaultCommand(new DefaultFeederCommand(feeder));
        configureButtonBindings();
    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }

    public ShooterSubsystem getShooter(){
        return shooter;
    }

    public IntakeSubsystem getIntake(){
        return intake;
    }

    public ClimberSubsystem getClimber(){
        return climber;
    }

    public FeederSubsystem getFeeder(){
        return feeder;
    }

    public VisionSubsystem getVision() { return vision; }

    public XboxController getController() { return controller; }

    public void configureButtonBindings() {
        new Button(() -> controller.getLeftTriggerAxis() > 0.5).whileHeld(new SimpleIntakeCommand(intake));
        new Button(() -> controller.getRightTriggerAxis() > 0.5).whileHeld(new AlignRobotToShootCommand(drivetrain, vision));
        new Button(() -> controller.getRightBumper()).whileHeld(new ManualShootCommand(feeder, shooter));
        new Button(() -> controller.getYButton()).whenPressed(new ZeroClimberCommand(climber));
        new Button(() -> controller.getXButton()).whenPressed(new ZeroHoodCommand(shooter));
//        //manual hood adjustment - 0: up, 180: down
//        new Button(() -> controller.getPOV() == 180.0).whenPressed(() -> shooter.setHoodTargetPosition(
//                shooter.getHoodAngle() + Constants.HOOD_MANUAL_ADJUST_INTERVAL)
//        );
//
//        new Button(() -> controller.getPOV() == 0.0).whenPressed(() -> shooter.setHoodTargetPosition(
//                shooter.getHoodAngle() - Constants.HOOD_MANUAL_ADJUST_INTERVAL)
//        );
//
//        //manual flywheel adjustment - 90: right, 270: left
//        new Button(() -> controller.getPOV() == 90.0).whenPressed(() -> shooter.setTargetFlywheelSpeed(
//                shooter.getTargetFlywheelSpeed() + Constants.FLYWHEEL_MANUAL_ADJUST_INTERVAL)
//        );
//
//        new Button(() -> controller.getPOV() == 270.0).whenPressed(() -> shooter.setTargetFlywheelSpeed(
//                shooter.getTargetFlywheelSpeed() - Constants.FLYWHEEL_MANUAL_ADJUST_INTERVAL)
//        );
    }
}
