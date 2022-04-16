package org.frcteam2910.c2022.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;

public class TargetingSolution implements Interpolatable<TargetingSolution> {
    public final double hoodAngle;
    public final double shooterVelocity;
    public final double estimatedAirTime;
    public final Rotation2d rotation;
    public final Translation2d targetPosition;

    public TargetingSolution(double hoodAngle, double shooterVelocity, double estimatedAirTime, Rotation2d rotation,
            Translation2d targetPosition) {
        this.hoodAngle = hoodAngle;
        this.shooterVelocity = shooterVelocity;
        this.estimatedAirTime = estimatedAirTime;
        this.rotation = rotation;
        this.targetPosition = targetPosition;
    }

    @Override
    public TargetingSolution interpolate(TargetingSolution endValue, double t) {
        return new TargetingSolution(MathUtil.interpolate(hoodAngle, endValue.hoodAngle, t),
                MathUtil.interpolate(shooterVelocity, endValue.shooterVelocity, t),
                MathUtil.interpolate(estimatedAirTime, endValue.estimatedAirTime, t),
                rotation.interpolate(endValue.rotation, t), targetPosition.interpolate(endValue.targetPosition, t));
    }
}
