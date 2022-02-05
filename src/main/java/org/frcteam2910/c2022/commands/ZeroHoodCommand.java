package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class ZeroHoodCommand extends CommandBase {
    private final double ZERO_HOOD_VElOCITY_TIME = 250; //in ms, 5 sec

    private ShooterSubsystem shooterSubsystem;

    private double zeroHoodStartTime;

    public ZeroHoodCommand(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setHoodZeroed(false);
        shooterSubsystem.setHoodVoltage(-2.0);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(shooterSubsystem.getHoodVoltage()) > 0.2) {
            zeroHoodStartTime = System.currentTimeMillis();
        }

        if (System.currentTimeMillis() - zeroHoodStartTime >= ZERO_HOOD_VElOCITY_TIME) {
            shooterSubsystem.setHoodZeroed(true);
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setHoodVoltage(0.0);
        shooterSubsystem.zeroHoodMotor();
    }
}