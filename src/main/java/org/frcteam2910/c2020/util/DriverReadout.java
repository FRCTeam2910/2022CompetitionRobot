package org.frcteam2910.c2020.util;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.util.Side;

public class DriverReadout {
    private final SendableChooser<Side> loadingBayChooser = new SendableChooser<>();

    public DriverReadout(RobotContainer container) {
        ShuffleboardTab tab = Shuffleboard.getTab("Driver Readout");

        loadingBayChooser.setDefaultOption("Left", Side.LEFT);
        loadingBayChooser.addOption("Right", Side.RIGHT);

        tab.addNumber("Pressure", () -> container.getSuperstructure().getCurrentPressure()).withSize(2, 2)
                .withPosition(0, 0).withWidget(BuiltInWidgets.kDial);
        tab.add("Autonomous Mode", container.getAutonomousChooser().getAutonomousModeChooser()).withSize(2, 1)
                .withPosition(2, 0);
        tab.add("Loading Station Bay", loadingBayChooser).withSize(2, 1).withPosition(2, 1);

        tab.add("Zero Gyroscope", new ZeroGyroscope(container.getDrivetrainSubsystem())).withSize(2, 1).withPosition(4,
                0);
    }

    public Side getSelectedLoadingBay() {
        return loadingBayChooser.getSelected();
    }

    private static class ZeroGyroscope extends CommandBase {
        private final DrivetrainSubsystem drivetrain;

        public ZeroGyroscope(DrivetrainSubsystem drivetrain) {
            this.drivetrain = drivetrain;

            setName("Zero Gyroscope");
        }

        @Override
        public void initialize() {
            drivetrain.resetGyroAngle(Rotation2.ZERO);
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}
