package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class SetHoodAngleCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final double targetAngle;
    private final boolean climbing;
    private final boolean fast;

    public SetHoodAngleCommand(ShooterSubsystem shooter, double targetAngle) {
        this(shooter, targetAngle, true, false);
    }

    public SetHoodAngleCommand(ShooterSubsystem shooter, double targetAngle, boolean fast) {
        this(shooter, targetAngle, fast, false);
    }

    public SetHoodAngleCommand(ShooterSubsystem shooter, double targetAngle, boolean fast, boolean climbing) {
        this.shooter = shooter;
        this.targetAngle = targetAngle;
        this.climbing = climbing;
        this.fast = fast;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setFastHoodConfig(fast);
        shooter.setHoodTargetPosition(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return shooter.isHoodAtTargetAngle(climbing);
    }
}