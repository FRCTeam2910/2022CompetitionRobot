package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class ZeroHoodCommand extends CommandBase {
    private static final double ZERO_HOOD_VELOCITY_TIME = 0.5; // in sec
    private static final double HOOD_MAX_ANGLE = 90;
    private static final double HOOD_VOLTAGE = 2.5;

    private static final double HOOD_ALLOWABLE_ZERO_VELOCITY = Math.toRadians(0.1);

    private final ShooterSubsystem shooterSubsystem;

    private double zeroHoodStartTime = Double.NaN;
    private boolean forward;

    public ZeroHoodCommand(ShooterSubsystem shooterSubsystem, boolean forward) {
        this.shooterSubsystem = shooterSubsystem;
        this.forward = forward;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setHoodZeroed(false);
        zeroHoodStartTime = Double.NaN;
        if (forward) {
            shooterSubsystem.setHoodVoltage(HOOD_VOLTAGE);
        } else {
            shooterSubsystem.setHoodVoltage(-HOOD_VOLTAGE);
        }
    }

    @Override
    public boolean isFinished() {
        if (Double.isFinite(zeroHoodStartTime)) {
            if (Math.abs(shooterSubsystem.getHoodVelocity()) > HOOD_ALLOWABLE_ZERO_VELOCITY) {
                zeroHoodStartTime = Double.NaN;
            } else
                return Timer.getFPGATimestamp() - zeroHoodStartTime >= ZERO_HOOD_VELOCITY_TIME;
        } else {
            if (Math.abs(shooterSubsystem.getHoodVelocity()) < HOOD_ALLOWABLE_ZERO_VELOCITY) {
                zeroHoodStartTime = Timer.getFPGATimestamp();
            }
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setHoodVoltage(0.0);
        if (!interrupted) {
            shooterSubsystem.setHoodZeroed(true);
            if (forward) {
                shooterSubsystem.setHoodMotorSensorPosition(HOOD_MAX_ANGLE);
            } else {
                shooterSubsystem.setHoodMotorSensorPosition(Math.toRadians(-1.0));
            }
        }
    }
}