package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;
import org.frcteam2910.c2020.subsystems.VisionSubsystem;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

public class TargetWithShooterCommand extends CommandBase {
    private static final InterpolatingTreeMap<InterpolatingDouble, Vector2> SHOOTER_TUNING = new InterpolatingTreeMap<>();

    private static final double MAXIMUM_ALLOWABLE_ANGLE_RANGE = Math.toRadians(3);
    private static final double MAXIMUM_ALLOWABLE_VELOCITY_RANGE = 50;

    private final ShooterSubsystem shooterSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final XboxController primaryController;

    static {
        SHOOTER_TUNING.put(new InterpolatingDouble(88.0), new Vector2(Math.toRadians(45.0), 3400));
        SHOOTER_TUNING.put(new InterpolatingDouble(100.0), new Vector2(Math.toRadians(43.0), 3400));
        SHOOTER_TUNING.put(new InterpolatingDouble(112.0), new Vector2(Math.toRadians(42.0), 3600));
        SHOOTER_TUNING.put(new InterpolatingDouble(124.0), new Vector2(Math.toRadians(39.0), 3600));
        SHOOTER_TUNING.put(new InterpolatingDouble(136.0), new Vector2(Math.toRadians(38.0), 3700));
        SHOOTER_TUNING.put(new InterpolatingDouble(148.0), new Vector2(Math.toRadians(37.0), 3750));
        SHOOTER_TUNING.put(new InterpolatingDouble(160.0), new Vector2(Math.toRadians(36.5), 3900));
        SHOOTER_TUNING.put(new InterpolatingDouble(172.0), new Vector2(Math.toRadians(35.5), 4000));
        SHOOTER_TUNING.put(new InterpolatingDouble(184.0), new Vector2(Math.toRadians(34.5), 4100));
        SHOOTER_TUNING.put(new InterpolatingDouble(196.0), new Vector2(Math.toRadians(32.5), 4200));
        SHOOTER_TUNING.put(new InterpolatingDouble(208.0), new Vector2(Math.toRadians(31.5), 4400));
        SHOOTER_TUNING.put(new InterpolatingDouble(220.0), new Vector2(Math.toRadians(30.0), 4600));
        SHOOTER_TUNING.put(new InterpolatingDouble(232.0), new Vector2(Math.toRadians(29.0), 4700));
        SHOOTER_TUNING.put(new InterpolatingDouble(244.0), new Vector2(Math.toRadians(29.0), 4800));
        SHOOTER_TUNING.put(new InterpolatingDouble(256.0), new Vector2(Math.toRadians(29.0), 4850));
        SHOOTER_TUNING.put(new InterpolatingDouble(268.0), new Vector2(Math.toRadians(29.0), 4900));
        SHOOTER_TUNING.put(new InterpolatingDouble(280.0), new Vector2(Math.toRadians(29.0), 4900));
        SHOOTER_TUNING.put(new InterpolatingDouble(292.0), new Vector2(Math.toRadians(29.0), 4950));
        SHOOTER_TUNING.put(new InterpolatingDouble(304.0), new Vector2(Math.toRadians(29.0), 5050));
        SHOOTER_TUNING.put(new InterpolatingDouble(316.0), new Vector2(Math.toRadians(29.0), 5100));
        SHOOTER_TUNING.put(new InterpolatingDouble(328.0), new Vector2(Math.toRadians(29.0), 5150));
    }

    public TargetWithShooterCommand(ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem,
            XboxController primaryController) {
        this.shooterSubsystem = shooterSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.primaryController = primaryController;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setFlywheelCurrentLimitEnabled(false);
    }

    @Override
    public void execute() {
        Vector2 angleAndSpeed = SHOOTER_TUNING
                .getInterpolated(new InterpolatingDouble(visionSubsystem.getDistanceToTarget().orElse(0.0)));

        shooterSubsystem.shootFlywheel(angleAndSpeed.y);
        shooterSubsystem.setHoodTargetAngle(angleAndSpeed.x);
        if (MathUtils.epsilonEquals(shooterSubsystem.getFlywheelVelocity(), angleAndSpeed.y,
                MAXIMUM_ALLOWABLE_VELOCITY_RANGE)
                && MathUtils.epsilonEquals(shooterSubsystem.getHoodAngle(), angleAndSpeed.x,
                        MAXIMUM_ALLOWABLE_ANGLE_RANGE)) {
            primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
        } else {
            primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setFlywheelCurrentLimitEnabled(true);
        shooterSubsystem.stopFlywheel();
        primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
    }
}
