package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class EnableCompressorCommand extends CommandBase {
    private final PneumaticHub pneumatics;

    public EnableCompressorCommand(PneumaticHub pneumatics) {
        this.pneumatics = pneumatics;
    }

    @Override
    public void initialize() {
        pneumatics.enableCompressorAnalog(115, 120);
    }
}
