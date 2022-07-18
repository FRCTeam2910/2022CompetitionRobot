package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;

public class StopShooterCommand extends CommandBase {
    private final ShooterSubsystem shooter;

    public StopShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.stopFlywheel();
    }
}
