package org.frcteam2910.c2020.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.VisionSubsystem;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Limelight;

public class VisionRotateToTargetCommand extends CommandBase {
    private static final PidConstants PID_CONSTANTS = new PidConstants(0.5, 2.0, 0.025);
    private static final double ROTATION_STATIC_CONSTANT = 0.3;
    private static final double MAXIMUM_AVERAGE_VELOCITY = 2.0;

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem visionSubsystem;

    private final DoubleSupplier xAxis;
    private final DoubleSupplier yAxis;

    private PidController controller = new PidController(PID_CONSTANTS);
    private double lastTime = 0.0;

    public VisionRotateToTargetCommand(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem,
            DoubleSupplier xAxis, DoubleSupplier yAxis) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        addRequirements(drivetrain);
        addRequirements(visionSubsystem);

        controller.setInputRange(0.0, 2.0 * Math.PI);
        controller.setContinuous(true);
        controller.setIntegralRange(Math.toRadians(10.0));
    }

    @Override
    public void initialize() {
        lastTime = Timer.getFPGATimestamp();
        visionSubsystem.setCamMode(Limelight.CamMode.VISION);
        visionSubsystem.setSnapshotEnabled(true);
        controller.reset();
    }

    @Override
    public void execute() {
        double time = Timer.getFPGATimestamp();
        double dt = time - lastTime;
        lastTime = time;

        Vector2 translationalVelocity = new Vector2(xAxis.getAsDouble(), yAxis.getAsDouble());

        double rotationalVelocity = 0.0;
        if (visionSubsystem.hasTarget()) {
            double currentAngle = drivetrain.getPose().rotation.toRadians();
            double targetAngle = visionSubsystem.getAngleToTarget().getAsDouble();
            controller.setSetpoint(targetAngle);
            rotationalVelocity = controller.calculate(currentAngle, dt);

            if (drivetrain.getAverageAbsoluteValueVelocity() < MAXIMUM_AVERAGE_VELOCITY) {
                rotationalVelocity += Math.copySign(ROTATION_STATIC_CONSTANT / RobotController.getBatteryVoltage(),
                        rotationalVelocity);
            }
        }
        drivetrain.drive(translationalVelocity, rotationalVelocity, true);
    }

    @Override
    public void end(boolean interrupted) {
        visionSubsystem.setSnapshotEnabled(false);
        drivetrain.drive(Vector2.ZERO, 0, false);
    }
}
