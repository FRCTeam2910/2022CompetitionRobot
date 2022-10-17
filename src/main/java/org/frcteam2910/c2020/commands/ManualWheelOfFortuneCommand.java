package org.frcteam2910.c2020.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.WheelOfFortuneSubsystem;

public class ManualWheelOfFortuneCommand extends CommandBase {
    private final WheelOfFortuneSubsystem wheelOfFortuneSubsystem;
    private final DoubleSupplier speedSupplier;

    public ManualWheelOfFortuneCommand(WheelOfFortuneSubsystem wheelOfFortuneSubsystem, DoubleSupplier speedSupplier) {
        this.wheelOfFortuneSubsystem = wheelOfFortuneSubsystem;
        this.speedSupplier = speedSupplier;

        addRequirements(wheelOfFortuneSubsystem);
    }

    @Override
    public void initialize() {
        wheelOfFortuneSubsystem.extendSolenoid();
    }

    @Override
    public void execute() {
        wheelOfFortuneSubsystem.setMotorSpeed(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        wheelOfFortuneSubsystem.stopMotor();
        wheelOfFortuneSubsystem.retractSolenoid();
    }
}
