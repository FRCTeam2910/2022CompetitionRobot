package org.frcteam2910.c2022.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2022.subsystems.FeederSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;
import org.frcteam2910.c2022.subsystems.VisionSubsystem;

public class AutoShootCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;
    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final PIDController controller = new PIDController(1.0, 0.0, 0.0);

    private final double SHOOTER_SPEED_VOLTS = 6;
    private final double FEEDER_SPEED = 0.5;

    public AutoShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, DrivetrainSubsystem drivetrain, VisionSubsystem vision){
        this.shooter = shooter;
        this.feeder = feeder;
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(feeder);
    }

    @Override
    public void execute() {
        if(vision.shooterHasTargets()){
            if(vision.isOnTarget()){
                feeder.setFeederSpeed(FEEDER_SPEED);
                shooter.setVoltage(SHOOTER_SPEED_VOLTS);
            } else {
                double setPoint = vision.getShooterAngleToTarget();
                double currentAngle = drivetrain.getPose().getRotation().getRadians();
                double rotationalVelocity = controller.calculate(currentAngle, setPoint);
                drivetrain.drive( new ChassisSpeeds(0.0, 0.0, rotationalVelocity) );
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setFeederSpeed(0.0);
        shooter.setVoltage(0.0);
    }
}