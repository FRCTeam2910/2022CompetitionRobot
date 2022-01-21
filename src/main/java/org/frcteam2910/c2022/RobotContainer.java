package org.frcteam2910.c2022;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frcteam2910.c2022.subsystems.ElevatorSubsystem;

public class RobotContainer {

    private final ElevatorSubsystem elevator = new ElevatorSubsystem();

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(elevator);
    }
}
