package org.frcteam2910.c2022.lib;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IDrivetrain extends Subsystem {
    /**
     * Gets the current pose of the drivetrain. The coordinates are such that 0,0 is
     * in the center of the field. +X is down field, and +Y is to the left if
     * looking down field.
     *
     * @return The current pose of the drivetrain.
     */
    Pose2d getCurrentPose();

    Optional<Pose2d> getPreviousPose(double timestamp);

    /**
     * Gets the field-oriented velocity of the drivetrain. X is how fast the robot
     * is moving down field.
     *
     * @return the current velocity of the drivetrain.
     */
    ChassisSpeeds getCurrentVelocity();

    Optional<Rotation2d> getTargetRotation();

    default double getTargetRotationAcceptableError() {
        return Math.toDegrees(5.0);
    }

    void setTargetVelocity(ChassisSpeeds targetVelocity);

    void setTargetVelocityAndRotation(ChassisSpeeds targetVelocity, Rotation2d targetRotation);

    void addVisionMeasurement(double captureTimestamp, Pose2d pose);

    default boolean isRotationAtTarget() {
        return getTargetRotation().map(targetRotation -> {
            double error = MathUtil
                    .angleModulus(getCurrentPose().getRotation().getRadians() - targetRotation.getRadians());

            return Math.abs(error) < Math.toDegrees(5.0);
        }).orElse(false);
    }
}
