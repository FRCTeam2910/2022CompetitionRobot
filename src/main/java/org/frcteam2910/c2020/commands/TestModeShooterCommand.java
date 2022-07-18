package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;

public class TestModeShooterCommand extends CommandBase {
    private final ShooterSubsystem shooter;

    public TestModeShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.stopFlywheel();
        shooter.disableHood();
    }
}
