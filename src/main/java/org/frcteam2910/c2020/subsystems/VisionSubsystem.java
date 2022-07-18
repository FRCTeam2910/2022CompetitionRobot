package org.frcteam2910.c2020.subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Limelight;

public class VisionSubsystem implements Subsystem {
    private static final double TARGET_HEIGHT = 98.25;
    private static final double LIMELIGHT_HEIGHT = 24.1;

    private static final double INNER_TARGET_RANGE_ANGLE = Math.toRadians(10.0); // "Theoretical": 18.0
    private static final double INNER_TARGET_DEPTH = 29.25;
    // The distance from the inner target to the apex of the triangle we use to find
    // the distance
    private static final double DISTANCE_FROM_INNER_TO_APEX = 16.92;

    private static final double LIMELIGHT_MOUNTING_ANGLE = Math.toRadians(34.0);

    private static final double TARGET_ALLOWABLE_ERROR = Math.toRadians(2.0);

    private static final Limelight LIMELIGHT = new Limelight();
    private final DrivetrainSubsystem drivetrain;

    private final NetworkTableEntry distanceToTargetEntry;
    private final NetworkTableEntry dXOuterEntry;
    private final NetworkTableEntry dYOuterEntry;
    private final NetworkTableEntry canSeeInnerTargetEntry;

    private boolean hasTarget;
    private boolean isInnerTargetVisible;
    private OptionalDouble distanceToTarget = OptionalDouble.empty();
    private OptionalDouble angleToTarget = OptionalDouble.empty();

    public VisionSubsystem(DrivetrainSubsystem drivetrainSubsystem) {
        drivetrain = drivetrainSubsystem;
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        distanceToTargetEntry = tab.add("distance to target", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
        dXOuterEntry = tab.add("dXOuter", 0.0).withPosition(1, 0).withSize(1, 1).getEntry();
        dYOuterEntry = tab.add("dYOuter", 0.0).withPosition(2, 0).withSize(1, 1).getEntry();
        canSeeInnerTargetEntry = tab.add("can see inner target", false).withPosition(3, 0).withSize(1, 1).getEntry();
        tab.addNumber("target angle", () -> Math.toDegrees(getAngleToTarget().orElse(Double.NaN))).withPosition(4, 0)
                .withSize(1, 1);
        tab.addBoolean("Is on target", this::isOnTarget).withPosition(5, 0).withSize(1, 1);
        tab.addNumber("Horizontal Target Error", () -> {
            double gyroAngle = drivetrain.getPose().rotation.toRadians();
            return getDistanceToTarget().orElse(0.0)
                    * (Math.sin(gyroAngle - getAngleToTarget().orElse(0.0)) / Math.sin(Math.PI / 2.0 - gyroAngle));
        }).withPosition(6, 0).withSize(1, 1);
    }

    public void setCamMode(Limelight.CamMode mode) {
        LIMELIGHT.setCamMode(mode);
    }

    public OptionalDouble getDistanceToTarget() {
        return distanceToTarget;
    }

    public OptionalDouble getAngleToTarget() {
        return angleToTarget;
    }

    public OptionalDouble getHorizontalError() {
        OptionalDouble distanceToTargetOpt = getDistanceToTarget();
        OptionalDouble angleToTargetOpt = getAngleToTarget();

        if (distanceToTargetOpt.isEmpty() || angleToTargetOpt.isEmpty()) {
            return OptionalDouble.empty();
        }

        double gyroAngle = drivetrain.getPose().rotation.toRadians();
        return OptionalDouble.of(distanceToTargetOpt.getAsDouble()
                * (Math.sin(gyroAngle - angleToTargetOpt.getAsDouble()) / Math.sin(Math.PI / 2.0 - gyroAngle)));
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public boolean isInnerTargetVisible() {
        return isInnerTargetVisible;
    }

    @Override
    public void periodic() {
        // Determine whether the Limelight has a target or not
        hasTarget = LIMELIGHT.hasTarget();
        if (hasTarget) {
            // Calculate the distance to the outer target
            Vector2 targetPosition = LIMELIGHT.getTargetPosition();
            double theta = LIMELIGHT_MOUNTING_ANGLE + targetPosition.y;
            double distanceToOuterTarget = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(theta);

            // Get the field oriented angle for the outer target, with latency compensation
            double angleToOuter = drivetrain
                    .getPoseAtTime(Timer.getFPGATimestamp() - LIMELIGHT.getPipelineLatency() / 1000.0).rotation
                            .toRadians()
                    - targetPosition.x;
            double dYOuter = distanceToOuterTarget * Math.sin(angleToOuter);
            double dXOuter = distanceToOuterTarget * Math.cos(angleToOuter);
            dXOuterEntry.setDouble(dXOuter);
            dYOuterEntry.setDouble(dYOuter);

            // Calculate the distance to the inner target
            double dXInner = dXOuter + INNER_TARGET_DEPTH;
            double distanceToInnerTarget = Math.hypot(dXInner, dYOuter);
            // Add DISTANCE_FROM_INNER_TO_APEX to dXInner here because we want if we did it
            // when we defined dXInner
            // distanceToInnerTarget would be incorrect, and we only need this extra bit to
            // determine if we can see
            // the inner target
            double angleToApex = Math.atan(dYOuter / (dXInner + DISTANCE_FROM_INNER_TO_APEX));
            if (angleToApex < 0.0) {
                angleToApex += 2 * Math.PI;
            }
            double angleToInner = Math.atan(dYOuter / dXInner);
            if (angleToInner < 0.0) {
                angleToInner += 2 * Math.PI;
            }

            // Check whether we can see the inner target
            isInnerTargetVisible = angleToApex <= INNER_TARGET_RANGE_ANGLE
                    || angleToApex >= 2 * Math.PI - INNER_TARGET_RANGE_ANGLE;
            if (isInnerTargetVisible) {
                distanceToTarget = OptionalDouble.of(distanceToInnerTarget);
            } else {
                distanceToTarget = OptionalDouble.of(distanceToOuterTarget);
            }
            angleToTarget = OptionalDouble.of(angleToOuter);
        } else {
            distanceToTarget = OptionalDouble.empty();
            angleToTarget = OptionalDouble.empty();
            isInnerTargetVisible = false;
        }

        // Update shuffleboard
        distanceToTargetEntry.setDouble(distanceToTarget.orElse(-1.0));
        canSeeInnerTargetEntry.setBoolean(isInnerTargetVisible);
    }

    public boolean isOnTarget() {
        OptionalDouble targetAngle = getAngleToTarget();
        if (targetAngle.isEmpty()) {
            return false;
        }

        double delta = targetAngle.getAsDouble() - drivetrain.getPose().rotation.toRadians();
        if (delta > Math.PI) {
            delta = 2.0 * Math.PI - delta;
        }

        return MathUtils.epsilonEquals(delta, 0, TARGET_ALLOWABLE_ERROR);
    }

    public void setLedMode(Limelight.LedMode mode) {
        LIMELIGHT.setLedMode(mode);
    }

    public void setSnapshotEnabled(boolean isEnabled) {
        LIMELIGHT.setSnapshotsEnabled(isEnabled);
    }
}
