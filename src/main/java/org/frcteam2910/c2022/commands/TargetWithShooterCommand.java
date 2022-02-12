package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;
import org.frcteam2910.c2022.subsystems.VisionSubsystem;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

public class TargetWithShooterCommand extends CommandBase {
    private final VisionSubsystem vision;
    private final ShooterSubsystem shooter;

    private static final InterpolatingTreeMap<InterpolatingDouble, Vector2> SHOOTER_TUNING = new InterpolatingTreeMap<>();

    static {

        SHOOTER_TUNING.put(new InterpolatingDouble(100.0), new Vector2(Math.toRadians(50), 3000));

    }
    public TargetWithShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        Vector2 angleAndSpeed = SHOOTER_TUNING
                .getInterpolated(new InterpolatingDouble(vision.getShooterDistanceToTarget()));

        shooter.setTargetFlywheelSpeed(angleAndSpeed.y);
        shooter.setHoodTargetPosition(angleAndSpeed.x);
    }
}
