package org.frcteam2910.c2022.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import georegression.struct.point.Point2D_F64;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.MovingAverage;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;

public class VisionSubsystem implements Subsystem {
    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(104);

    private static final double TARGET_ALLOWABLE_ERROR = Math.toRadians(2.5);
    private static final double LIMELIGHT_FORWARD = Units.inchesToMeters(5.22);
    private static final double LIMELIGHT_MOUNTING_ANGLE = Math.toRadians(45.0);
    private static final double LIMELIGHT_HEIGHT = Units.inchesToMeters(17.5);

    private static final double CAMERA_HEIGHT_PIXELS = 720;
    private static final double CAMERA_WIDTH_PIXELS = 960;

    private static final double HORIZONTAL_FOV = Math.toRadians(59.6);
    private static final double VERTICAL_FOV = Math.toRadians(45.7);

    private final DrivetrainSubsystem drivetrain;

    // private final PhotonCamera shooterLimelight = new PhotonCamera("gloworm");

    private boolean shooterHasTargets = false;
    private double distanceToTarget = Double.NaN;
    private double distanceToTargetX = Double.NaN;
    private double distanceToTargetY = Double.NaN;
    private double angleToTarget = Double.NaN;
    private MovingAverage averageDistance = new MovingAverage(10);
    private double theta;
    private double centerXPixels;
    private double centerYPixels;
    private double centerXAngle;
    private double centerYAngle;

    private PhotonPipelineResult result;

    public VisionSubsystem(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        tab.addBoolean("shooter has targets", () -> shooterHasTargets).withPosition(0, 0).withSize(1, 1);
        tab.addNumber("distance to target", () -> distanceToTarget).withPosition(1, 0).withSize(1, 1);
        tab.addNumber("X", () -> distanceToTargetX).withPosition(1, 1).withSize(1, 1);
        tab.addNumber("Y", () -> distanceToTargetY).withPosition(1, 2).withSize(1, 1);
        tab.addNumber("angle to target", () -> Units.radiansToDegrees(angleToTarget)).withPosition(2, 0).withSize(1, 1);
        tab.addNumber("avg distance to target", () -> averageDistance.get()).withPosition(3, 0).withSize(1, 1);
        tab.addNumber("theta", () -> Math.toDegrees(theta));
        tab.addNumber("center X Pixels", () -> centerXPixels);
        tab.addNumber("center Y Pixels", () -> centerYPixels);
        tab.addNumber("center X Angle", () -> Math.toDegrees(centerXAngle));
        tab.addNumber("center Y Angle", () -> Math.toDegrees(centerYAngle));
        tab.addNumber("angle using atan2",
                () -> Math.toDegrees(Math.atan2(getDistanceToTarget().y, getDistanceToTarget().x)));
        tab.addBoolean("Is Vision on Target", this::isOnTarget);
    }

    public Vector2 getDistanceToTarget() {
        return new Vector2(distanceToTargetX, distanceToTargetY);
    }

    public double getAngleToTarget() {
        return angleToTarget;
    }

    public boolean shooterHasTargets() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) > 0.5;
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
        boolean hasTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) > 0.5;
        // result = shooterLimelight.getLatestResult();

        double pitch = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double yaw = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

        if (hasTarget) {
            theta = Math.toRadians(pitch) + LIMELIGHT_MOUNTING_ANGLE;
            distanceToTarget = (TARGET_HEIGHT_METERS - LIMELIGHT_HEIGHT) / Math.tan(theta);
            angleToTarget = drivetrain.getPose().getRotation().getRadians() + Math.toRadians(yaw);
            distanceToTargetX = distanceToTarget * Math.cos(drivetrain.getPose().getRotation().getRadians());
            distanceToTargetY = distanceToTarget * Math.sin(drivetrain.getPose().getRotation().getRadians());
        }

        // result = shooterLimelight.getLatestResult();
        //
        // if (result != null && result.hasTargets()) {
        // List<PhotonTrackedTarget> targets = result.getTargets();
        // List<Point2D_F64> bottomCorners = new ArrayList<>();
        // List<Point2D_F64> topCorners = new ArrayList<>();
        // for (int i = 0; i < targets.size(); i++) {
        // List<TargetCorner> corners = targets.get(i).getCorners();
        // corners.sort((a, b) -> Double.compare(a.y, b.y));
        //
        // bottomCorners.add(convertToPoint(corners.get(0)));
        // bottomCorners.add(convertToPoint(corners.get(1)));
        //
        // topCorners.add(convertToPoint(corners.get(2)));
        // topCorners.add(convertToPoint(corners.get(3)));
        // }
        //
        // FitEllipseAlgebraic_F64 topEllipse = new FitEllipseAlgebraic_F64();
        // if (topEllipse.process(topCorners)) {
        // double a = topEllipse.getEllipse().A;
        // double c = topEllipse.getEllipse().C;
        // double d = topEllipse.getEllipse().D;
        // double e = topEllipse.getEllipse().E;
        //
        // centerXPixels = d / 2 * a;
        // centerYPixels = -e / 2 * c;
        //
        // centerXAngle = ((centerXPixels / CAMERA_WIDTH_PIXELS) - 0.5) *
        // HORIZONTAL_FOV;
        // centerYAngle = -((centerYPixels / CAMERA_HEIGHT_PIXELS) - 0.5) *
        // VERTICAL_FOV;
        //
        // theta = centerYAngle + LIMELIGHT_MOUNTING_ANGLE;
        // distanceToTarget = (TARGET_HEIGHT_METERS - LIMELIGHT_HEIGHT) /
        // Math.tan(theta);
        // averageDistance.add(distanceToTarget);
        // angleToTarget = drivetrain.getPose().getRotation().getRadians() +
        // centerXAngle;
        // }
        // }
    }

    private Point2D_F64 convertToPoint(TargetCorner corner) {
        return new Point2D_F64(corner.x, corner.y);
    }
}