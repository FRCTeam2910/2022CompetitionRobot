package org.frcteam2910.c2022.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.Robot;
import org.frcteam2910.c2022.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2022.subsystems.VisionSubsystem;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;

public class AlignRobotToShootCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final PidController controller = new PidController(new PidConstants(1.0, 0.0, 0.0));

    public AlignRobotToShootCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        controller.setInputRange(0, 2 * Math.PI);
        controller.setContinuous(true);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        controller.reset();
    }

    @Override
    public void execute() {
        double setPoint = Math.atan2(drivetrain.getPose().getY(), drivetrain.getPose().getY());
        double currentAngle = drivetrain.getPose().getRotation().getRadians();

        controller.setSetpoint(setPoint);
        double rotationalVelocity = controller.calculate(currentAngle, Robot.kDefaultPeriod);
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, rotationalVelocity));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}