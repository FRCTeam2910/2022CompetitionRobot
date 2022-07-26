package org.frcteam2910.c2021;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frcteam2910.c2021.vision.Vision;
import org.frcteam2910.c2021.vision.VisionIOLimelight;

public class Robot extends TimedRobot {
    private final Vision vision = new Vision(new VisionIOLimelight());

    @Override
    public void robotInit() {
        vision.register();

        vision.setTranslationConsumer(translation -> {
            SmartDashboard.putNumber("X", Units.metersToInches(translation.translation.getX()));
            SmartDashboard.putNumber("Y", Units.metersToInches(translation.translation.getY()));
        });
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
