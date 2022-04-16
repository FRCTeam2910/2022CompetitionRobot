// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.littletonrobotics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.util.TunableNumber;

/**
 * Constants for the vision camera.
 */
public final class VisionConstants {
    public static final int widthPixels = 960;
    public static final int heightPixels = 720;
    public static final double crosshairX = 14.4;
    public static final double crosshairY = 0.0;
    public static final Rotation2d fovHorizontal = Rotation2d.fromDegrees(59.6);
    public static final Rotation2d fovVertical = Rotation2d.fromDegrees(49.7);

    public static final double vehicleToCameraX = Units.inchesToMeters(11.788);
    public static final double vehicleToCameraY = 0.0;
    public static final double vehicleToCameraZ = Units.inchesToMeters(34.648);
    public static final Rotation2d cameraVerticalRotation = Rotation2d.fromDegrees(41.173); // Measured relative to the
                                                                                            // flat part of the hood
    public static final TunableNumber cameraVerticalRotationFudgeDegrees = new TunableNumber(
            "VisionConstants/FudgeDegrees", -0.4);

    public static CameraPosition getCameraPosition() {
        // Side-on frame of reference (y is used as z)
        Pose2d vehicleToCamera = new Pose2d(vehicleToCameraX, vehicleToCameraZ, new Rotation2d());
        vehicleToCamera = vehicleToCamera.transformBy(
                new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0).minus(cameraVerticalRotation)
                        .minus(Rotation2d.fromDegrees(cameraVerticalRotationFudgeDegrees.get()))));

        // Convert to camera position
        return new CameraPosition(vehicleToCamera.getY(),
                Rotation2d.fromDegrees(180.0).minus(vehicleToCamera.getRotation()), new Transform2d(
                        new Translation2d(vehicleToCamera.getX(), vehicleToCameraY), Rotation2d.fromDegrees(180.0)));
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
