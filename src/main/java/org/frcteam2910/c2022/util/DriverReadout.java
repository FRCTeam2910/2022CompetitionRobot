package org.frcteam2910.c2022.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import org.frcteam2910.c2022.Constants;
import org.frcteam2910.c2022.RobotContainer;

public class DriverReadout {
    private final RobotContainer container;

    private final Field2d field = new Field2d();
    private final FieldObject2d targetObject = field.getObject("Target");
    private final FieldObject2d goalObject = field.getObject("Goal");

    public DriverReadout(RobotContainer container) {
        this.container = container;
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME);

        tab.addNumber("Pressure", () -> container.getSuperstructure().getCurrentPressure()).withSize(2, 2)
                .withPosition(0, 0).withWidget(BuiltInWidgets.kDial);
        tab.add("Autonomous Mode", container.getAutonomousChooser().getModeChooser()).withSize(2, 1).withPosition(2, 0);
        tab.addCamera("Camera", "Camera", "http://limelight.local:5800", "http://10.29.10.11:5800").withSize(3, 3)
                .withPosition(4, 0);
        tab.add("Field", field);
    }

    private Pose2d transformPose(Pose2d pose) {
        return new Pose2d(pose.getX() + Units.feetToMeters(27.0), pose.getY() + Units.feetToMeters(13.5),
                pose.getRotation());
    }

    public void update() {
        var pose = container.getDrivetrain().getCurrentPose();

        field.setRobotPose(transformPose(pose));
    }

    public void setGoalPosition(Translation2d position) {
        goalObject.setPose(transformPose(new Pose2d(position, new Rotation2d())));
    }

    public void setTargetPosition(Translation2d position) {
        targetObject.setPose(transformPose(new Pose2d(position, new Rotation2d())));
    }
}
