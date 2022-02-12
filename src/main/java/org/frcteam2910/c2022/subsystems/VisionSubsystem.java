package org.frcteam2910.c2022.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2022.Robot;
import org.frcteam2910.common.math.MathUtils;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem implements Subsystem {
    final double CAMERA_HEIGHT_METERS = 0.0; // find
    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(104);
    final double CAMERA_PITCH_RADIANS = 0.0; // find
    final double SHOOTER_LIMELIGHT_BASE_ANGLE = 0.0; // find, lowest angle of hood then add pitch
    final double GOAL_RANGE_UNIT = 0.0; // find
    private static final double TARGET_ALLOWABLE_ERROR = Math.toRadians(2.5);

    private final DrivetrainSubsystem drivetrain;

    private final PhotonCamera shooterLimelight = new PhotonCamera("photonvision");

    private boolean shooterHasTargets = false;
    private double shooterDistanceToTarget = Double.NaN;
    private double shooterAngleToTarget = Double.NaN;

    private PhotonPipelineResult result;

    public VisionSubsystem(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
    }

    public double getShooterDistanceToTarget() {
        return shooterDistanceToTarget;
    }

    public double getShooterAngleToTarget() {
        return shooterAngleToTarget;
    }

    public boolean shooterHasTargets() {
        return result.hasTargets();
    }

    public boolean isOnTarget() {
        shooterHasTargets = shooterHasTargets();
        if (shooterHasTargets) {
            double delta = shooterAngleToTarget - drivetrain.getPose().getRotation().getRadians();
            if (delta > Math.PI) {
                delta = 2.0 * Math.PI - delta;
            }

            return MathUtils.epsilonEquals(delta, 0, TARGET_ALLOWABLE_ERROR);
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation())
            return;
        result = shooterLimelight.getLatestResult();
        if (result.hasTargets()) {
            shooterDistanceToTarget = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS,
                    TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));

            PhotonTrackedTarget target = shooterLimelight.getLatestResult().getBestTarget();
            shooterAngleToTarget = drivetrain.getPose().getRotation().getRadians() + Math.toRadians(target.getYaw());
        } else {
            shooterDistanceToTarget = Double.NaN;
            shooterAngleToTarget = Double.NaN;
        }
    }
}