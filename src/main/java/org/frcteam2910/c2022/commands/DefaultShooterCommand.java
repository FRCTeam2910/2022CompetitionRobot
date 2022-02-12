package org.frcteam2910.c2022.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier voltageSupplier;
    private final DoubleSupplier hoodVoltageSupplier;

    public DefaultShooterCommand(ShooterSubsystem shooter, DoubleSupplier voltageSupplier,
            DoubleSupplier hoodVoltageSupplier) {
        this.shooter = shooter;
        this.voltageSupplier = voltageSupplier;
        this.hoodVoltageSupplier = hoodVoltageSupplier;
        addRequirements(shooter);
    }

    public void execute() {
        shooter.setFlywheelVoltage(voltageSupplier.getAsDouble());
        shooter.setHoodVoltage(hoodVoltageSupplier.getAsDouble());
    }
}
