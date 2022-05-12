package org.frcteam2910.c2022;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
import org.frcteam2910.c2022.commands.*;
import org.frcteam2910.c2022.subsystems.*;

public class RobotContainer {
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

    private final XboxController controller = new XboxController(Constants.CONTROLLER_PORT);

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(drivetrain);

        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, this::getForwardInput, this::getStrafeInput,
                this::getRotationInput));
        configureButtonBindings();
    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }

    public XboxController getController() {
        return controller;
    }

    public void configureButtonBindings() {
        new Button(controller::getBackButton).whenPressed(drivetrain::zeroRotation);
    }

    private static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance)
            return 0.0;

        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }

    private double getForwardInput() {
        return -square(deadband(controller.getLeftY(), 0.1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    }

    private double getStrafeInput() {
        return -square(deadband(controller.getLeftX(), 0.1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    }

    private double getRotationInput() {
        return -square(deadband(controller.getRightX(), 0.1))
                * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }
}
