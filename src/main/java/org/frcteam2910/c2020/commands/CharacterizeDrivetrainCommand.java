package org.frcteam2910.c2020.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class CharacterizeDrivetrainCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;

    private final NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault()
            .getEntry("/SmartDashboard/SysIdAutoSpeed");
    private final NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault()
            .getEntry("/SmartDashboard/SysIdTelemetry");

    private List<Double> telemetryData = new ArrayList<>();

    private double priorAutospeed = 0.0;

    public CharacterizeDrivetrainCommand(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetGyroAngle(Rotation2.ZERO);
        drivetrain.resetPose(RigidTransform2.ZERO);

        NetworkTableInstance.getDefault().setUpdateRate(10.0e-3);
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        double position = drivetrain.getPose().translation.x;
        double velocity = drivetrain.getVelocity().x;

        double battery = RobotController.getBatteryVoltage();
        double motorVoltage = battery * Math.abs(priorAutospeed);

        double autospeed = autoSpeedEntry.getDouble(0.0);
        priorAutospeed = autospeed;

        drivetrain.drive(new Vector2(autospeed, 0.0), 0.0, false);

        telemetryData.add(now);
        telemetryData.add(autospeed * RobotController.getInputVoltage());
        telemetryData.add(position);
        telemetryData.add(velocity);

    }

    @Override
    public void end(boolean interrupted) {
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < telemetryData.size(); ++i) {
            if (i != 0)
                b.append(", ");
            b.append(telemetryData.get(i));
        }

        telemetryEntry.setString(b.toString());

        drivetrain.drive(Vector2.ZERO, 0.0, false);
    }
}
