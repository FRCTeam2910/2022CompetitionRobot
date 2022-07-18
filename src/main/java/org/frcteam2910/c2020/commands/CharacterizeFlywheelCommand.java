package org.frcteam2910.c2020.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;

public class CharacterizeFlywheelCommand extends CommandBase {
    private final ShooterSubsystem shooter;

    private final NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault()
            .getEntry("/SmartDashboard/SysIdAutoSpeed");
    private final NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault()
            .getEntry("/SmartDashboard/SysIdTelemetry");

    private final List<Double> telemetry = new ArrayList<>();

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

        telemetry.add(now);
        telemetry.add(autospeed * RobotController.getInputVoltage());
        telemetry.add(position);
        telemetry.add(velocity);

    }

    @Override
    public void end(boolean interrupted) {
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < telemetry.size(); ++i) {
            if (i != 0)
                b.append(", ");
            b.append(telemetry.get(i));
        }

        telemetryEntry.setString(b.toString());

        shooter.stopFlywheel();
    }
}
