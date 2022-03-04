package org.frcteam2910.c2022.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;
import org.frcteam2910.c2022.subsystems.VisionSubsystem;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

public class TargetWithShooterCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;

    private static final InterpolatingTreeMap<InterpolatingDouble, Vector2> SHOOTER_TUNING = new InterpolatingTreeMap<>();

    static {

        SHOOTER_TUNING.put(new InterpolatingDouble(1.0),
                new Vector2(Math.toRadians(3.0), Units.rotationsPerMinuteToRadiansPerSecond(1850)));
        SHOOTER_TUNING.put(new InterpolatingDouble(1.5),
                new Vector2(Math.toRadians(6.5), Units.rotationsPerMinuteToRadiansPerSecond(1900)));
        SHOOTER_TUNING.put(new InterpolatingDouble(2.00),
                new Vector2(Math.toRadians(10.5), Units.rotationsPerMinuteToRadiansPerSecond(1950)));
        SHOOTER_TUNING.put(new InterpolatingDouble(2.5),
                new Vector2(Math.toRadians(13.5), Units.rotationsPerMinuteToRadiansPerSecond(2000)));
        SHOOTER_TUNING.put(new InterpolatingDouble(3.0),
                new Vector2(Math.toRadians(15.0), Units.rotationsPerMinuteToRadiansPerSecond(2050)));
        SHOOTER_TUNING.put(new InterpolatingDouble(3.5),
                new Vector2(Math.toRadians(19.0), Units.rotationsPerMinuteToRadiansPerSecond(2100)));
        SHOOTER_TUNING.put(new InterpolatingDouble(4.0),
                new Vector2(Math.toRadians(23.0), Units.rotationsPerMinuteToRadiansPerSecond(2200)));
        SHOOTER_TUNING.put(new InterpolatingDouble(4.5),
                new Vector2(Math.toRadians(26.0), Units.rotationsPerMinuteToRadiansPerSecond(2300)));
        SHOOTER_TUNING.put(new InterpolatingDouble(5.0),
                new Vector2(Math.toRadians(30.0), Units.rotationsPerMinuteToRadiansPerSecond(2400)));
        SHOOTER_TUNING.put(new InterpolatingDouble(5.5),
                new Vector2(Math.toRadians(31.5), Units.rotationsPerMinuteToRadiansPerSecond(2500)));
        SHOOTER_TUNING.put(new InterpolatingDouble(6.0),
                new Vector2(Math.toRadians(33.0), Units.rotationsPerMinuteToRadiansPerSecond(2550)));

    }
    public TargetWithShooterCommand(ShooterSubsystem shooter, DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        this.shooter = shooter;
        this.drivetrain = drivetrain;
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
