package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class ZeroHoodCommand extends CommandBase {
    private static final double ZERO_HOOD_VELOCITY_TIME = 0.25; // in sec
    private static final double HOOD_VOLTAGE = 1.5;
    private static final double HOOD_ALLOWABLE_ZERO_VELOCITY = Math.toRadians(0.1);

    private final ShooterSubsystem shooter;

    private final boolean forward;
    private final boolean flywheel;

    private double zeroHoodStartTime = Double.NaN;

    public ZeroHoodCommand(ShooterSubsystem shooter, boolean forward) {
        this(shooter, forward, false);
    }

    public ZeroHoodCommand(ShooterSubsystem shooter, boolean forward, boolean flywheel) {
        this.shooter = shooter;
        this.forward = forward;
        this.flywheel = flywheel;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setHoodZeroed(false);
        zeroHoodStartTime = Double.NaN;

        if (forward) {
            shooter.setHoodVoltage(HOOD_VOLTAGE);
        } else {
            shooter.setHoodVoltage(-HOOD_VOLTAGE);
        }

        if (flywheel) {
            shooter.setTargetFlywheelSpeed(ShooterSubsystem.FLYWHEEL_IDLE_SPEED);
            shooter.enableCurrentLimits(true);
        }
    }

    @Override
    public boolean isFinished() {
        if (Double.isFinite(zeroHoodStartTime)) {
            if (Math.abs(shooter.getHoodVelocity()) > HOOD_ALLOWABLE_ZERO_VELOCITY) {
                zeroHoodStartTime = Double.NaN;
            } else {
                return Timer.getFPGATimestamp() - zeroHoodStartTime >= ZERO_HOOD_VELOCITY_TIME;
            }
        } else {
            if (Math.abs(shooter.getHoodVelocity()) < HOOD_ALLOWABLE_ZERO_VELOCITY) {
                zeroHoodStartTime = Timer.getFPGATimestamp();
            }
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setHoodVoltage(0.0);
        if (!interrupted) {
            shooter.setHoodZeroed(true);
            if (forward) {
                shooter.setHoodMotorSensorPosition(ShooterSubsystem.HOOD_MAX_ANGLE);
            } else {
                shooter.setHoodMotorSensorPosition(ShooterSubsystem.HOOD_MIN_ANGLE - Math.toRadians(0.5));
            }
        }
        shooter.enableCurrentLimits(false);
    }
}