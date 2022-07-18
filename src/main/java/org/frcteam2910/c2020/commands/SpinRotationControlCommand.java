package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.WheelOfFortuneSubsystem;

import static org.frcteam2910.c2020.subsystems.WheelOfFortuneSubsystem.SPINNER_REVOLUTIONS_PER_WHEEL_SECTION;

public class SpinRotationControlCommand extends CommandBase {
    private static final double ROTATION_CONTROL_NUM_SECTIONS = 27.0;
    private static final double NUM_SECTIONS_ALLOWABLE_ERROR = 0.5;

    private WheelOfFortuneSubsystem spinner;

    public SpinRotationControlCommand(WheelOfFortuneSubsystem wheelOfFortuneSpinner) {
        spinner = wheelOfFortuneSpinner;
        addRequirements(spinner);
    }

    @Override
    public void initialize() {
        spinner.resetEncoderPosition();
        spinner.spin(ROTATION_CONTROL_NUM_SECTIONS * SPINNER_REVOLUTIONS_PER_WHEEL_SECTION);
    }

    @Override
    public boolean isFinished() {
        return spinner.getEncoderPosition() >= (ROTATION_CONTROL_NUM_SECTIONS - NUM_SECTIONS_ALLOWABLE_ERROR)
                * SPINNER_REVOLUTIONS_PER_WHEEL_SECTION;
    }

    @Override
    public void end(boolean interrupted) {
        spinner.stopMotor();
    }
}
