// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frcteam2910.c2021;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Constants for the vision camera. */
public final class VisionConstants {
    public static final int widthPixels = 960;
    public static final int heightPixels = 720;
    public static final Rotation2d fovHorizontal = Rotation2d.fromDegrees(59.6);
    public static final Rotation2d fovVertical = Rotation2d.fromDegrees(49.7);

    private static final double cameraHeight = Units.inchesToMeters(23.0);
    private static final double offsetX = Units.inchesToMeters(0.0);
    private static final double verticalRotation = Math.toRadians(26.0);

    private static final double upperCameraHeight = Units.inchesToMeters(42.5);
    private static final double upperOffsetX = Units.inchesToMeters(9.0);

    public static CameraPosition getCameraPosition() {
        return new CameraPosition(cameraHeight, new Rotation2d(verticalRotation),
                new Transform2d(new Translation2d(offsetX, 0.0), Rotation2d.fromDegrees(0.0)));
    }

    public static final class CameraPosition {
        public final double cameraHeight;
        public final Rotation2d verticalRotation;
        public final Transform2d vehicleToCamera;

        public CameraPosition(double cameraHeight, Rotation2d verticalRotation, Transform2d vehicleToCamera) {
            this.cameraHeight = cameraHeight;
            this.verticalRotation = verticalRotation;
            this.vehicleToCamera = vehicleToCamera;
        }
    }
}
