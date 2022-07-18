package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;

public class HomeHoodMotorCommand extends CommandBase {

    private final long HOOD_ZERO_VELOCITY_TIME_PERIOD = 250;// in ms

    private ShooterSubsystem shooterSubsystem;

    private long zeroVelocityTimestamp;

    public HomeHoodMotorCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setHoodHomed(false);
        shooterSubsystem.setHoodMotorPower(0.1);
        zeroVelocityTimestamp = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {

        if (Math.abs(shooterSubsystem.getHoodVelocity()) > Math.toRadians(0.01)) {
            zeroVelocityTimestamp = System.currentTimeMillis();
        }

        if (System.currentTimeMillis() - zeroVelocityTimestamp >= HOOD_ZERO_VELOCITY_TIME_PERIOD) {
            shooterSubsystem.setHoodHomed(true);
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.zeroHoodMotor();
        shooterSubsystem.setHoodMotorPower(0.0);
    }
}
