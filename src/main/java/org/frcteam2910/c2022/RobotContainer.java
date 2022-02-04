package org.frcteam2910.c2022;

import edu.wpi.first.wpilibj.Joystick;
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

    private final Joystick joystick = new Joystick(0);
    private final XboxController controller = new XboxController(Constants.CONTROLLER_PORT);

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(climber);
        CommandScheduler.getInstance().registerSubsystem(shooter);
        CommandScheduler.getInstance().registerSubsystem(intake);
        CommandScheduler.getInstance().registerSubsystem(feeder);
        CommandScheduler.getInstance().registerSubsystem(vision);

        shooter.setDefaultCommand(new DefaultShooterCommand(shooter,
                () -> joystick.getRawAxis(0) * 12,
                () -> joystick.getRawAxis(1) * 12));
        CommandScheduler.getInstance().setDefaultCommand(intake,
                new DefaultIntakeCommand(intake));
        CommandScheduler.getInstance().setDefaultCommand(climber,
                new DefaultClimberCommand(climber,
                () -> joystick.getRawAxis(2),
                climber.getPID()));
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain,
            () -> joystick.getRawAxis(3) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> joystick.getRawAxis(4) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> joystick.getRawAxis(5) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));
        configureButtonBindings();
    }

    public ClimberSubsystem getClimber() {
        return climber;
    }

    public void configureButtonBindings() {
//        new Button(() -> joystick.getRawButton(1)).whenPressed(new ClimberToPointCommand(climber, 1.0));
//        new Button(() -> joystick.getRawButton(2)).whenPressed(new ClimberToPointCommand(climber, 0.1));
//        // new Button(() -> joystick.getRawButton(3)).whenPressed(new InstantCommand(() -> climber.setPositionControl(false)));
//        new Button(() -> joystick.getRawButton(4)).whenPressed(new InstantCommand(() -> climber.setPositionControl(true)));
//        new Button(() -> joystick.getRawButton(3)).whenPressed(new SequentialCommandGroup(
//                new ClimberToPointCommand(climber, 1.1),
//                new WaitCommand(0.5),
//                new ClimberToPointCommand(climber, 0.1),
//                new WaitCommand(0.5),
//                new ClimberToPointCommand(climber, 1.1)
        new Button(() -> controller.getLeftTriggerAxis() > 0.5).whileHeld(new SimpleIntakeCommand(intake, feeder, 1.0));
//        new Button(() -> controller.getRightTriggerAxis() > 0.5).whileHeld(new AutoShootCommand);
//        new Button(() -> controller.getRightBumper()).whileHeld(new ManualShootCommand(shooter));
    }
}
