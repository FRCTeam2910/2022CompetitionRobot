package org.frcteam2910.c2022;

import edu.wpi.first.wpilibj.PneumaticHub;

public class Superstructure {
    private final PneumaticHub pneumatics = new PneumaticHub();

    public Superstructure() {
        pneumatics.enableCompressorAnalog(115, 120);
    }

    public double getCurrentPressure() {
        return pneumatics.getPressure(0);
    }
}
