package org.frcteam2910.c2022.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class ClimbChooser {
    private final SendableChooser<ClimbType> climbChooser = new SendableChooser<>();

    public ClimbChooser() {
        climbChooser.setDefaultOption("Traversal Partway", ClimbType.TRAVERSAL_PARTWAY);
        climbChooser.addOption("Traversal Hook", ClimbType.TRAVERSAL_HOOK);
        climbChooser.addOption("Traversal Partway", ClimbType.TRAVERSAL_PARTWAY);
        climbChooser.addOption("High Hook", ClimbType.HIGH_HOOK);
        climbChooser.addOption("High Partway", ClimbType.HIGH_PARTWAY);
        climbChooser.addOption("Mid / Low", ClimbType.MID);
    }

    public SendableChooser<ClimbType> getClimbChooser() {
        return climbChooser;
    }

    public enum ClimbType {
        TRAVERSAL_HOOK, TRAVERSAL_PARTWAY, HIGH_HOOK, HIGH_PARTWAY, MID
    }
}
