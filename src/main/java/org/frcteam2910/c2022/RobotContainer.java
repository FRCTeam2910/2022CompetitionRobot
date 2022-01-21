package org.frcteam2910.c2022;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frcteam2910.c2022.subsystems.ElevatorSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;
import org.frcteam2910.c2022.commands.DefaultShooterCommand;

public class RobotContainer {

    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final Joystick joystick = new Joystick(0);
    private final ShooterSubsystem shooter = new ShooterSubsystem();

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(elevator);
        CommandScheduler.getInstance().registerSubsystem(shooter);
        shooter.setDefaultCommand(new DefaultShooterCommand(shooter,
                () -> joystick.getRawAxis(0) * 12,
                () -> joystick.getRawAxis(1) * 12));
    }
}
