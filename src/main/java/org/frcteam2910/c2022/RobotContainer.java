package org.frcteam2910.c2022;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frcteam2910.c2022.commands.*;
import org.frcteam2910.c2022.subsystems.FeederSubsystem;
import org.frcteam2910.c2022.subsystems.IntakeSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;
import org.frcteam2910.c2022.commands.DefaultShooterCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import org.frcteam2910.c2022.commands.ClimberToPointCommand;
import org.frcteam2910.c2022.commands.DefaultClimberCommand;
import org.frcteam2910.c2022.subsystems.ClimberSubsystem;

import java.time.Instant;

public class RobotContainer {

    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final FeederSubsystem feeder = new FeederSubsystem();

    private final Joystick joystick = new Joystick(0);
    private final XboxController controller = new XboxController(Constants.CONTROLLER_PORT);

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(climber);
        CommandScheduler.getInstance().registerSubsystem(shooter);
        CommandScheduler.getInstance().registerSubsystem(intake);
        CommandScheduler.getInstance().registerSubsystem(feeder);

        shooter.setDefaultCommand(new DefaultShooterCommand(shooter,
                () -> joystick.getRawAxis(0) * 12,
                () -> joystick.getRawAxis(1) * 12));
        CommandScheduler.getInstance().setDefaultCommand(intake,
                new DefaultIntakeCommand(intake));
        CommandScheduler.getInstance().setDefaultCommand(climber,
                new DefaultClimberCommand(climber,
                () -> joystick.getRawAxis(2),
                climber.getPID()
        ));
        configureButtonBindings();
    }

    public ClimberSubsystem getClimber() {
        return climber;
    }

    public void configureButtonBindings() {
        new Button(() -> joystick.getRawButton(1)).whileHeld(new ClimberToPointCommand(climber, 1.0));
        new Button(() -> joystick.getRawButton(2)).whileHeld(new ClimberToPointCommand(climber, 0.1));
//        new Button(() -> joystick.getRawButton(3)).whenPressed(new InstantCommand(() -> climber.setPositionControl(false)));
        new Button(() -> joystick.getRawButton(4)).whenPressed(new InstantCommand(() -> climber.setPositionControl(true)));
        new Button(() -> joystick.getRawButton(3)).whenPressed(new SequentialCommandGroup(
                new ClimberToPointCommand(climber, 1.1),
                new WaitCommand(0.5),
                new ClimberToPointCommand(climber, 0.1),
                new WaitCommand(0.5),
                new ClimberToPointCommand(climber, 1.1)
        ));
    }
}
