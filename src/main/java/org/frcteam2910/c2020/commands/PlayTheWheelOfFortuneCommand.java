package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.WheelOfFortuneSubsystem;

public class PlayTheWheelOfFortuneCommand extends CommandBase {
    private final WheelOfFortuneSubsystem wheelOfFortune;
    private double lastPosition = 0.0;

    public PlayTheWheelOfFortuneCommand(WheelOfFortuneSubsystem wheelOfFortune) {
        this.wheelOfFortune = wheelOfFortune;

        addRequirements(wheelOfFortune);
    }

    @Override
    public void initialize() {
        lastPosition = wheelOfFortune.getEncoderPosition();
    }

    @Override
    public void execute() {
        wheelOfFortune.setMotorSpeed(0.75);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(wheelOfFortune.getEncoderPosition() - lastPosition) >= 4.0 * (9.0 * (32.0 / 2.0));
    }

    @Override
    public void end(boolean interrupted) {
        wheelOfFortune.stopMotor();
    }
}
