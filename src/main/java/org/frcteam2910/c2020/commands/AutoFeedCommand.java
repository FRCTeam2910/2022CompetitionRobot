package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

public class AutoFeedCommand extends CommandBase {
    private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> AUTO_FEEDER_OUTPUT_MAP = new InterpolatingTreeMap<>();
    private static final double TIME_UNTIL_FEED = 0.05;
    private DrivetrainSubsystem drivetrainSubsystem;
    private FeederSubsystem feederSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private VisionSubsystem visionSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private Timer timer = new Timer();

    static {
        AUTO_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(2580.0), new InterpolatingDouble(1.0));
        AUTO_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(3873.0), new InterpolatingDouble(1.0));
        AUTO_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(4368.0), new InterpolatingDouble(0.7));
        AUTO_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(4763.0), new InterpolatingDouble(0.6));
        AUTO_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(4965.0), new InterpolatingDouble(0.5));
        AUTO_FEEDER_OUTPUT_MAP.put(new InterpolatingDouble(5412.0), new InterpolatingDouble(0.5));
    }

    public AutoFeedCommand(DrivetrainSubsystem drivetrain, FeederSubsystem feeder, ShooterSubsystem shooter,
            VisionSubsystem vision, IntakeSubsystem intake) {
        this.drivetrainSubsystem = drivetrain;
        this.feederSubsystem = feeder;
        this.shooterSubsystem = shooter;
        this.visionSubsystem = vision;
        this.intakeSubsystem = intake;
    }

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
    }

    @Override
    public void execute() {
        if (visionSubsystem.isOnTarget() && shooterSubsystem.isFlywheelAtTargetVelocity()
                && shooterSubsystem.isHoodAtTargetAngle()) {
            timer.start();
            if (timer.hasElapsed(TIME_UNTIL_FEED)) {
                feederSubsystem.spinMotor(AUTO_FEEDER_OUTPUT_MAP
                        .getInterpolated(new InterpolatingDouble(shooterSubsystem.getFlywheelVelocity())).value);
                if (feederSubsystem.isFifthBallAtIntake()) {
                    intakeSubsystem.setMotorOutput(0.5);
                } else {
                    intakeSubsystem.setMotorOutput(0.0);
                }
            } else {
                feederSubsystem.spinMotor(0.0);
                intakeSubsystem.setMotorOutput(0.0);
            }
        } else {
            feederSubsystem.spinMotor(0.0);
            intakeSubsystem.setMotorOutput(0.0);
            timer.stop();
            timer.reset();
        }

    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.spinMotor(0.0);
        intakeSubsystem.setMotorOutput(0.0);
    }
}
