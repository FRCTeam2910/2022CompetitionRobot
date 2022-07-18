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
        // 2d port
        // SHOOTER_TUNING.put(new InterpolatingDouble(127.6),new
        // Vector2(Math.toRadians(43.0),2979));
        // SHOOTER_TUNING.put(new InterpolatingDouble(185.0),new
        // Vector2(Math.toRadians(35.5),3228));
        // SHOOTER_TUNING.put(new InterpolatingDouble(258.5),new
        // Vector2(Math.toRadians(27.0),3774));
        // SHOOTER_TUNING.put(new InterpolatingDouble(315.0),new
        // Vector2(Math.toRadians(22.0),4469));

        // power port
        // SHOOTER_TUNING.put(new InterpolatingDouble(1.0),new
        // Vector2(Math.toRadians(25.24),4400));
        // SHOOTER_TUNING.put(new InterpolatingDouble(1000.0),new
        // Vector2(Math.toRadians(25.24),4400));

        // Good numbers for 3d port
        // SHOOTER_TUNING.put(new InterpolatingDouble(92.7),new
        // Vector2(Math.toRadians(46.7),2580));
        // SHOOTER_TUNING.put(new InterpolatingDouble(123.8),new
        // Vector2(Math.toRadians(44.2),2676));
        // SHOOTER_TUNING.put(new InterpolatingDouble(155.1),new
        // Vector2(Math.toRadians(35.2),3123));
        // SHOOTER_TUNING.put(new InterpolatingDouble(186.8),new
        // Vector2(Math.toRadians(28.1),3773));
        // SHOOTER_TUNING.put(new InterpolatingDouble(243.6),new
        // Vector2(Math.toRadians(24.6),4269));
        // SHOOTER_TUNING.put(new InterpolatingDouble(328.0),new
        // Vector2(Math.toRadians(19.2),4716));

        SHOOTER_TUNING.put(new InterpolatingDouble(98.0), new Vector2(Math.toRadians(48.95), 2580));
        SHOOTER_TUNING.put(new InterpolatingDouble(110.0), new Vector2(Math.toRadians(41.09), 2827));
        SHOOTER_TUNING.put(new InterpolatingDouble(122.6), new Vector2(Math.toRadians(38.59), 3026));
        SHOOTER_TUNING.put(new InterpolatingDouble(133.0), new Vector2(Math.toRadians(36.09), 3124));
        SHOOTER_TUNING.put(new InterpolatingDouble(144.0), new Vector2(Math.toRadians(34.09), 3272));
        SHOOTER_TUNING.put(new InterpolatingDouble(164.0), new Vector2(Math.toRadians(31.09), 3400));
        SHOOTER_TUNING.put(new InterpolatingDouble(173.7), new Vector2(Math.toRadians(29.09), 3520));
        SHOOTER_TUNING.put(new InterpolatingDouble(183.7), new Vector2(Math.toRadians(27.7), 3617));
        SHOOTER_TUNING.put(new InterpolatingDouble(200.0), new Vector2(Math.toRadians(26.59), 3724));
        SHOOTER_TUNING.put(new InterpolatingDouble(211.3), new Vector2(Math.toRadians(25.59), 3873));
        SHOOTER_TUNING.put(new InterpolatingDouble(228.0), new Vector2(Math.toRadians(23.59), 4022));
        SHOOTER_TUNING.put(new InterpolatingDouble(237.4), new Vector2(Math.toRadians(22.59), 4171));
        SHOOTER_TUNING.put(new InterpolatingDouble(253.5), new Vector2(Math.toRadians(20.89), 4320));
        SHOOTER_TUNING.put(new InterpolatingDouble(267.0), new Vector2(Math.toRadians(20.09), 4368));
        SHOOTER_TUNING.put(new InterpolatingDouble(280.0), new Vector2(Math.toRadians(19.09), 4467));
        SHOOTER_TUNING.put(new InterpolatingDouble(294.0), new Vector2(Math.toRadians(18.59), 4664));
        SHOOTER_TUNING.put(new InterpolatingDouble(307.0), new Vector2(Math.toRadians(18.09), 4763));
        SHOOTER_TUNING.put(new InterpolatingDouble(326.0), new Vector2(Math.toRadians(17.19), 4867));
        SHOOTER_TUNING.put(new InterpolatingDouble(345.0), new Vector2(Math.toRadians(16.69), 5000));
        SHOOTER_TUNING.put(new InterpolatingDouble(359.0), new Vector2(Math.toRadians(16.0), 5263));
        SHOOTER_TUNING.put(new InterpolatingDouble(371.7), new Vector2(Math.toRadians(15.19), 5412));
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
                && MathUtils.epsilonEquals(shooterSubsystem.getHoodMotorAngle(), angleAndSpeed.x,
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
