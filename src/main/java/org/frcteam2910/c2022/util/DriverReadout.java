package org.frcteam2910.c2022.util;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.frcteam2910.c2022.Constants;
import org.frcteam2910.c2022.RobotContainer;

public class DriverReadout {

    public DriverReadout(RobotContainer container) {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME);

        tab.addNumber("Pressure", () -> container.getSuperstructure().getCurrentPressure()).withSize(2, 2)
                .withPosition(0, 0).withWidget(BuiltInWidgets.kDial);
        tab.add("Autonomous Mode", container.getAutonomousChooser().getModeChooser()).withSize(2, 1).withPosition(2, 0);
        tab.add("Autonomous Mode", container.getClimbChooser().getClimbChooser()).withSize(2, 1).withPosition(4, 3);
        tab.addCamera("Camera", "Camera", "http://limelight.local:5800", "http://10.29.10.11:5800").withSize(3, 3)
                .withPosition(4, 0);
    }
}
