// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.littletonrobotics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful side points. All dimensions are
 * in meters.
 */
public final class FieldConstants {

    // Field dimensions
    public static final double fieldLength = Units.inchesToMeters(54.0 * 12.0);
    public static final double fieldWidth = Units.inchesToMeters(27.0 * 12.0);
    // Vision target
    public static final double visionTargetDiameter = Units.inchesToMeters(4.0 * 12.0 + 5.375);
    public static final double visionTargetHeightLower = Units.inchesToMeters(8.0 * 12.0 + 5.625); // Bottom of tape
    public static final double visionTargetHeightUpper = visionTargetHeightLower + Units.inchesToMeters(2.0); // Top of
                                                                                                                // tape

    // Dimensions of hub and tarmac
    public static final Translation2d hubCenter = new Translation2d(0.0, 0.0);
}
