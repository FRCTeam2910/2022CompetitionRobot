package org.frcteam2910.c2022.util;

import java.util.Set;

public class SysIdGeneralMechanismLogger extends SysIdLogger {
    private static final Set<String> MECHANISMS = Set.of("Arm", "Elevator", "Simple");
    private double primaryMotorVoltage = 0.0;

    public double getMotorVoltage() {
        return primaryMotorVoltage;
    }

    public void log(double measuredPosition, double measuredVelocity) {
        updateData();
        if (data.size() < DATA_BUFFER_SIZE) {
            data.add(timestamp);
            data.add(primaryMotorVoltage);
            data.add(measuredPosition);
            data.add(measuredVelocity);
        }

        primaryMotorVoltage = motorVoltage;
    }

    public void reset() {
        super.reset();
        primaryMotorVoltage = 0.0;
    }

    public boolean isWrongMechanism() {
        return !MECHANISMS.contains(mechanism);
    }
}
