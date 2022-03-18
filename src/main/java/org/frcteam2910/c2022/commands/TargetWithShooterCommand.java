package org.frcteam2910.c2022.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;
import org.frcteam2910.c2022.subsystems.VisionSubsystem;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

public class TargetWithShooterCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;

    private static final InterpolatingTreeMap<InterpolatingDouble, Vector2> SHOOTER_TUNING = new InterpolatingTreeMap<>();

    static {

        SHOOTER_TUNING.put(new InterpolatingDouble(1.0),
                new Vector2(Math.toRadians(6.0), Units.rotationsPerMinuteToRadiansPerSecond(1750)));
        SHOOTER_TUNING.put(new InterpolatingDouble(1.5),
                new Vector2(Math.toRadians(8.5), Units.rotationsPerMinuteToRadiansPerSecond(1850)));
        SHOOTER_TUNING.put(new InterpolatingDouble(2.0),
                new Vector2(Math.toRadians(11.0), Units.rotationsPerMinuteToRadiansPerSecond(1925)));
        SHOOTER_TUNING.put(new InterpolatingDouble(2.5),
                new Vector2(Math.toRadians(12.5), Units.rotationsPerMinuteToRadiansPerSecond(1975)));
        SHOOTER_TUNING.put(new InterpolatingDouble(3.0),
                new Vector2(Math.toRadians(14.5), Units.rotationsPerMinuteToRadiansPerSecond(2050)));
        SHOOTER_TUNING.put(new InterpolatingDouble(3.5),
                new Vector2(Math.toRadians(16.5), Units.rotationsPerMinuteToRadiansPerSecond(2125)));
        SHOOTER_TUNING.put(new InterpolatingDouble(4.0),
                new Vector2(Math.toRadians(19.0), Units.rotationsPerMinuteToRadiansPerSecond(2225)));
        SHOOTER_TUNING.put(new InterpolatingDouble(4.5),
                new Vector2(Math.toRadians(22.0), Units.rotationsPerMinuteToRadiansPerSecond(2275)));
        SHOOTER_TUNING.put(new InterpolatingDouble(5.0),
                new Vector2(Math.toRadians(24.0), Units.rotationsPerMinuteToRadiansPerSecond(2350)));
        SHOOTER_TUNING.put(new InterpolatingDouble(5.5),
                new Vector2(Math.toRadians(25.5), Units.rotationsPerMinuteToRadiansPerSecond(2450)));
        SHOOTER_TUNING.put(new InterpolatingDouble(6.0),
                new Vector2(Math.toRadians(27.5), Units.rotationsPerMinuteToRadiansPerSecond(2600)));

    }
    public TargetWithShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        // double distance = drivetrain.getPose().getTranslation().getNorm();
        double distance = vision.getDistanceToTarget();
        Vector2 angleAndSpeed = SHOOTER_TUNING.getInterpolated(new InterpolatingDouble(distance));

        shooter.setTargetFlywheelSpeed(angleAndSpeed.y);
        shooter.setHoodTargetPosition(angleAndSpeed.x);
    }
}
