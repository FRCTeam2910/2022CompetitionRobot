package org.frcteam2910.c2022;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.c2022.commands.EnableCompressorCommand;

public class Superstructure {
    private final PneumaticHub pneumatics = new PneumaticHub();

    public Superstructure() {
        pneumatics.enableCompressorAnalog(115, 120);
        SmartDashboard.putData(new EnableCompressorCommand(pneumatics));
    }

    public double getCurrentPressure() {
        return pneumatics.getPressure(0);
    }
}
