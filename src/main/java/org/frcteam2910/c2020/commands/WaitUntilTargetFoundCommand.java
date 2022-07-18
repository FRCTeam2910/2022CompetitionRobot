package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.VisionSubsystem;

public class WaitUntilTargetFoundCommand extends CommandBase {
    private final VisionSubsystem vision;

    public WaitUntilTargetFoundCommand(VisionSubsystem vision) {
        this.vision = vision;
    }

    @Override
    public boolean isFinished() {
        return vision.doesIntakeHaveTarget();
    }
}
