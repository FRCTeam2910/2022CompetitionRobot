package org.frcteam2910.c2021;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Button;
import org.frcteam2910.c2021.commands.DefaultDriveCommand;
import org.frcteam2910.c2021.commands.TargetCommand;
import org.frcteam2910.c2021.subsystems.DrivetrainSubsystem;
import org.frcteam2910.visionlib.TargetingLookupEntry;
import org.frcteam2910.visionlib.TargetingSolver;
import org.littletonrobotics.oi.OverrideOI;
import org.littletonrobotics.subsystems.Vision;
import org.littletonrobotics.subsystems.VisionIOLimelight;

public class RobotContainer {
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    private final Vision vision = new Vision(new VisionIOLimelight(), drivetrain);

    private final DriverReadout driverVisual = new DriverReadout(drivetrain);

    private final XboxController controller = new XboxController(0);

    public RobotContainer() {
        drivetrain.register();
        vision.register();
        vision.setSuppliers(() -> {
            boolean tuningMode = SmartDashboard.getBoolean("Vision/TuningMode", false);
            return tuningMode ? OverrideOI.VisionLedMode.ALWAYS_ON : OverrideOI.VisionLedMode.AUTO;
        });

        TargetingSolver solver = new TargetingSolver(new Translation2d());
        solver.addEntry(1.0,
                new TargetingLookupEntry(Math.toRadians(7.0), Units.rotationsPerMinuteToRadiansPerSecond(1725), 1.0));
        solver.addEntry(1.5,
                new TargetingLookupEntry(Math.toRadians(9.5), Units.rotationsPerMinuteToRadiansPerSecond(1725), 1.1));
        solver.addEntry(2.0,
                new TargetingLookupEntry(Math.toRadians(11.5), Units.rotationsPerMinuteToRadiansPerSecond(1750), 1.2));
        solver.addEntry(2.5,
                new TargetingLookupEntry(Math.toRadians(13.5), Units.rotationsPerMinuteToRadiansPerSecond(1825), 1.3));
        solver.addEntry(3.0,
                new TargetingLookupEntry(Math.toRadians(16.0), Units.rotationsPerMinuteToRadiansPerSecond(1900), 1.45));
        solver.addEntry(3.5,
                new TargetingLookupEntry(Math.toRadians(18.5), Units.rotationsPerMinuteToRadiansPerSecond(2000), 1.6));
        solver.addEntry(4.0,
                new TargetingLookupEntry(Math.toRadians(21.5), Units.rotationsPerMinuteToRadiansPerSecond(2100), 1.6));
        solver.addEntry(4.5,
                new TargetingLookupEntry(Math.toRadians(24.0), Units.rotationsPerMinuteToRadiansPerSecond(2175), 1.7));

        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, this::getForwardInput, this::getStrafeInput,
                this::getRotationInput));

        new Button(controller::getRightBumper).whileHeld(new TargetCommand(drivetrain, vision, solver, driverVisual,
                this::getForwardInput, this::getStrafeInput));
    }

    private static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance)
            return 0.0;

        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }

    private double getForwardInput() {
        return -square(deadband(controller.getLeftY(), 0.1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    }

    private double getStrafeInput() {
        return -square(deadband(controller.getLeftX(), 0.1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    }

    private double getRotationInput() {
        return -square(deadband(controller.getRightX(), 0.1))
                * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    public DriverReadout getVisual() {
        return driverVisual;
    }
}
