// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frcteam2910.c2021.vision;

/** Vision subsystem hardware interface. */
public interface VisionIO {
    /** The set of loggable inputs for the vision subsystem. */
    public static class VisionIOInputs {
        public double captureTimestamp = 0.0;
        public double[] cornerX = new double[]{};
        public double[] cornerY = new double[]{};
        public boolean simpleValid = false;
        public double simpleAngle = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {
    }

    /** Enabled or disabled vision LEDs. */
    public default void setLeds(boolean enabled) {
    }

}
