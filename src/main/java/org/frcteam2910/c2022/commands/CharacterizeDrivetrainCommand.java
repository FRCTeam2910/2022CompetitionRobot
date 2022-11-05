package org.frcteam2910.c2022.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2022.util.SysIdGeneralMechanismLogger;

public class CharacterizeDrivetrainCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;

    private final SysIdGeneralMechanismLogger logger = new SysIdGeneralMechanismLogger();

    public CharacterizeDrivetrainCommand(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setPose(new Pose2d()); // Reset the drivetrain position

        logger.initLogging();
    }

    @Override
    public void execute() {
        // Update the data
        double position = drivetrain.getPose().getX();
        double velocity = drivetrain.getCurrentVelocity().vxMetersPerSecond;
        logger.log(position, velocity);

        // Get the voltage
        double voltage = logger.getMotorVoltage();

        // Drive at new voltage
        drivetrain.drive(
                new ChassisSpeeds(voltage / 12.0 * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0.0, 0.0));
    }

    @Override
    public void end(boolean interrupted) {
        logger.sendData();
        logger.reset();
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
