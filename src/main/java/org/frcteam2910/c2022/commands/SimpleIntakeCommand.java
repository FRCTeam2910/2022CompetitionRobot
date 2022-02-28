package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2022.subsystems.FeederSubsystem;
import org.frcteam2910.c2022.subsystems.IntakeSubsystem;

public class SimpleIntakeCommand extends CommandBase {
    private static final double INTAKE_SPEED = 1.0;

    private final IntakeSubsystem intake;
    private final FeederSubsystem feeder;
    private final XboxController controller;

    public SimpleIntakeCommand(IntakeSubsystem intake, FeederSubsystem feeder, XboxController controller) {
        this.intake = intake;
        this.feeder = feeder;
        this.controller = controller;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setExtended(true);
    }

    @Override
    public void execute() {
        intake.setIntakeSpeed(INTAKE_SPEED);
        if (feeder.isBallAtEntry()) {
            controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
        } else {
            controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0.0);
        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
    }
}
