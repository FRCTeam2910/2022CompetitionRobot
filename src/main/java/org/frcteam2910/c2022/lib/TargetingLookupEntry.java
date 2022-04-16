package org.frcteam2910.c2022.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

public class TargetingLookupEntry implements Interpolatable<TargetingLookupEntry> {
    public final double hoodAngleRadians;
    public final double shooterVelocityRadiansPerSecond;

    public final double airTimeSeconds;

    public TargetingLookupEntry(double hoodAngleRadians, double shooterVelocityRadiansPerSecond,
            double airTimeSeconds) {
        this.hoodAngleRadians = hoodAngleRadians;
        this.shooterVelocityRadiansPerSecond = shooterVelocityRadiansPerSecond;
        this.airTimeSeconds = airTimeSeconds;
    }

    @Override
    public TargetingLookupEntry interpolate(TargetingLookupEntry endValue, double t) {
        return new TargetingLookupEntry(MathUtil.interpolate(hoodAngleRadians, endValue.hoodAngleRadians, t),
                MathUtil.interpolate(shooterVelocityRadiansPerSecond, endValue.shooterVelocityRadiansPerSecond, t),
                MathUtil.interpolate(airTimeSeconds, endValue.airTimeSeconds, t));
    }
}
