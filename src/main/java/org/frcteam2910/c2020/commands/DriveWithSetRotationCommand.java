package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.Axis;

public class DriveWithSetRotationCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private Axis forward;
    private Axis strafe;
    private double setRotation;

    private PidController rotationController = new PidController(new PidConstants(0.5, 0.0, 0.02));

    public DriveWithSetRotationCommand(DrivetrainSubsystem drivetrain, Axis forward, Axis strafe, double setRotation) {
        this.drivetrain = drivetrain;
        this.forward = forward;
        this.strafe = strafe;
        this.setRotation = setRotation;

        rotationController.setInputRange(0.0, 2 * Math.PI);
        rotationController.setContinuous(true);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotationController.reset();
        rotationController.setSetpoint(setRotation);
    }

    @Override
    public void execute() {
        double rotationOutput = rotationController.calculate(drivetrain.getPose().rotation.toRadians(), 0.02);
        drivetrain.drive(new Vector2(forward.get(true), strafe.get(true)), rotationOutput, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(Vector2.ZERO, 0.0, false);
    }
}
