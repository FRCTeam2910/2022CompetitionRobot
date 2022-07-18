package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;
import org.frcteam2910.c2020.subsystems.IntakeSubsystem;
import org.frcteam2910.common.robot.input.XboxController;

public class SimpleIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final FeederSubsystem feederSubsystem;
    private final double intakeSpeed;
    private double speed;
    private boolean isFifthBallAtIntake;
    private XboxController controller;

    public SimpleIntakeCommand(IntakeSubsystem intake, FeederSubsystem feederSubsystem, XboxController controller,
            double intakeSpeed, double feederSpeed) {
        this.intake = intake;
        this.intakeSpeed = intakeSpeed;
        this.feederSubsystem = feederSubsystem;
        this.speed = feederSpeed;
        this.isFifthBallAtIntake = false;
        this.controller = controller;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setTopExtended(true);

    }

    @Override
    public void execute() {
        if (feederSubsystem.isFifthBallAtIntake() && intake.hasIntakeMotorPassedCurrentThreshHold()) {// Stop spinning
                                                                                                        // the wheels if
                                                                                                        // there is a
                                                                                                        // 5th ball - we
                                                                                                        // dont want to
                                                                                                        // destroy the
                                                                                                        // ball
            intake.setMotorOutput(0.0);
            this.isFifthBallAtIntake = true;
            controller.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
        } else if (!isFifthBallAtIntake) {
            intake.setMotorOutput(intakeSpeed);// Else, function normally
        }

        if (feederSubsystem.shouldAdvance()) {// Move the feeder
            feederSubsystem.spinMotor(speed);
        } else {
            feederSubsystem.spinMotor(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.spinMotor(0.0);
        // intake.setTopExtended(this.isFifthBallAtIntake);//Keep the top intake down if
        // we have a fifth ball
        intake.setMotorOutput(0.0);
        this.isFifthBallAtIntake = false;
        controller.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
    }
}
