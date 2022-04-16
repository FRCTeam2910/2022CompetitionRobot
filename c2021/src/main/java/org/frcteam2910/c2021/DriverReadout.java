package org.frcteam2910.c2021;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import org.frcteam2910.visionlib.IDrivetrain;

public class DriverReadout {
    private final IDrivetrain drivetrain;

    private final Field2d field = new Field2d();

    private final FieldObject2d targetObject = field.getObject("Target");
    private final FieldObject2d goalObject = field.getObject("Goal");

    public DriverReadout(IDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        ShuffleboardTab visualTab = Shuffleboard.getTab("Visuals");
        visualTab.add("Field", field);
    }

    private Pose2d transformPose(Pose2d pose) {
        return new Pose2d(pose.getX() + Units.feetToMeters(27.0), pose.getY() + Units.feetToMeters(13.5),
                pose.getRotation());
    }

    public void update() {
        var pose = drivetrain.getCurrentPose();

        field.setRobotPose(transformPose(pose));
    }

    public void setGoalPosition(Translation2d position) {
        goalObject.setPose(transformPose(new Pose2d(position, new Rotation2d())));
    }

    public void setTargetPosition(Translation2d position) {
        targetObject.setPose(transformPose(new Pose2d(position, new Rotation2d())));
    }

    public Sendable getField() {
        return field;
    }
}
