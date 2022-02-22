package org.frcteam2910.c2022.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import georegression.fitting.curves.FitEllipseAlgebraic_F64;
import georegression.struct.point.Point2D_F64;
import org.frcteam2910.common.math.MathUtils;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class VisionSubsystem implements Subsystem {
    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(104);

    private static final double TARGET_ALLOWABLE_ERROR = Math.toRadians(2.5);
    private static final double LIMELIGHT_MOUNTING_ANGLE = 0.0;
    private static final double LIMELIGHT_HEIGHT = 0.0;

    private final DrivetrainSubsystem drivetrain;

    private final PhotonCamera shooterLimelight = new PhotonCamera("photonvision");

    private boolean shooterHasTargets = false;
    private double distanceToTarget = Double.NaN;
    private double angleToTarget = Double.NaN;

    private PhotonPipelineResult result;

    public VisionSubsystem(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter) {
        this.drivetrain = drivetrain;

        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        tab.addBoolean("shooter has targets", () -> shooterHasTargets).withPosition(0, 0).withSize(1, 1);
        tab.addNumber("distance to target", () -> distanceToTarget).withPosition(1, 0).withSize(1, 1);
        tab.addNumber("angle to target", () -> angleToTarget).withPosition(2, 0).withSize(1, 1);
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    public double getAngleToTarget() {
        return angleToTarget;
    }

    public boolean shooterHasTargets() {
        return result.hasTargets();
    }

    public boolean isOnTarget() {
        shooterHasTargets = shooterHasTargets();
        if (shooterHasTargets) {
            double delta = angleToTarget - drivetrain.getPose().getRotation().getRadians();
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
        List<PhotonTrackedTarget> targets = result.getTargets();
        List<Point2D_F64> bottomCorners = new ArrayList<>();
        List<Point2D_F64> topCorners = new ArrayList<>();
        for (int i = 0; i < targets.size(); i++) {
            List<TargetCorner> corners = targets.get(i).getCorners();
            corners.sort((a, b) -> Double.compare(a.y, b.y));

            bottomCorners.add(new Point2D_F64(corners.get(0).x, corners.get(0).y));
            bottomCorners.add(new Point2D_F64(corners.get(1).x, corners.get(1).y));

            topCorners.add(new Point2D_F64(corners.get(2).x, corners.get(2).y));
            topCorners.add(new Point2D_F64(corners.get(3).x, corners.get(3).y));
        }

        FitEllipseAlgebraic_F64 topEllipse = new FitEllipseAlgebraic_F64();
        if (topEllipse.process(topCorners)) {
            double a = topEllipse.getEllipse().A;
            double c = topEllipse.getEllipse().C;
            double d = topEllipse.getEllipse().D;
            double e = topEllipse.getEllipse().E;

            double centerX = d / 2 * a;
            double centerY = -e / 2 * c;

            double theta = LIMELIGHT_MOUNTING_ANGLE + centerY;
            distanceToTarget = (TARGET_HEIGHT_METERS - LIMELIGHT_HEIGHT) / Math.tan(theta);

            angleToTarget = drivetrain.getPose().getRotation().getRadians() + centerX;
        }
    }
}