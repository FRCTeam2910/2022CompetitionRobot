package org.frcteam2910.c2020.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;

public class CharacterizeFlywheelCommand extends CommandBase {
    private final ShooterSubsystem shooter;

    private final NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    private final NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

    private final Number[] telemetryData = new Number[6];

    private double priorAutospeed = 0.0;

    public CharacterizeFlywheelCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.resetFlywheelPosition();
        NetworkTableInstance.getDefault().setUpdateRate(10.0e-3);
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        double position = shooter.getFlywheelPosition();
        double velocity = shooter.getFlywheelVelocity();

        double battery = RobotController.getBatteryVoltage();
        double motorVoltage = battery * Math.abs(priorAutospeed);

        double autospeed = autoSpeedEntry.getDouble(0.0);
        priorAutospeed = autospeed;

        shooter.setFlywheelOutput(autospeed);

        telemetryData[0] = now;
        telemetryData[1] = battery;
        telemetryData[2] = autospeed;
        telemetryData[3] = motorVoltage;
        telemetryData[4] = position;
        telemetryData[5] = velocity;

        telemetryEntry.setNumberArray(telemetryData);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
    }
}
