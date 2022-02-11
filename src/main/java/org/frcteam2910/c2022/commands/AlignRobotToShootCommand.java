package org.frcteam2910.c2022.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2022.subsystems.FeederSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;
import org.frcteam2910.c2022.subsystems.VisionSubsystem;

public class AlignRobotToShootCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final PIDController controller = new PIDController(1.0, 0.0, 0.0);

    public AlignRobotToShootCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision){
        this.drivetrain = drivetrain;
        this.vision = vision;
    }

    @Override
    public void execute() {
        if(vision.shooterHasTargets()){
            double setPoint = vision.getShooterAngleToTarget();
            double currentAngle = drivetrain.getPose().getRotation().getRadians();
            double rotationalVelocity = controller.calculate(currentAngle, setPoint);
            drivetrain.drive( new ChassisSpeeds(0.0, 0.0, rotationalVelocity) );
        }
    }
}