package org.frcteam2910.c2020.util;

public enum DetectedColor {

    RED, YELLOW, BLUE, GREEN;

    // Based on which color the robot is detecting, the color the field sensor is
    // detecting will be the same
    // regardless of which side of the control panel the robot is facing.
    public DetectedColor getColorOnFieldSensor() throws UnsupportedOperationException {
        switch (this) {
            case RED :
                return BLUE;
            case YELLOW :
                return GREEN;
            case BLUE :
                return RED;
            case GREEN :
                return YELLOW;
            default :
                throw new UnsupportedOperationException("Unknown color");
        }
    }

    private DetectedColor getAdjacentColor(boolean clockwise) throws UnsupportedOperationException {
        switch (this) {
            case RED :
                return clockwise ? YELLOW : GREEN;
            case YELLOW :
                return clockwise ? BLUE : RED;
            case BLUE :
                return clockwise ? GREEN : YELLOW;
            case GREEN :
                return clockwise ? RED : BLUE;
            default :
                throw new UnsupportedOperationException("Unknown color");
        }
    }

    public int findNumSectionsAwayFromColor(DetectedColor desiredColor) {
        if (desiredColor == null || desiredColor == this) {
            return 0;
        } else if (getAdjacentColor(true) == desiredColor) {
            return 1;
        } else if (getAdjacentColor(false) == desiredColor) {
            return -1;
        } else {
            return 2;
        }
    }
    // 1 = 1 section away in the clockwise direction; -1 = 1 direction away
    // counterclockwise

    public static DetectedColor convertGameMessageToColor(String gameMessage) throws IllegalArgumentException {
        switch (gameMessage.charAt(0)) {
            case 'R' :
                return RED;
            case 'Y' :
                return YELLOW;
            case 'B' :
                return BLUE;
            case 'G' :
                return GREEN;
            default :
                throw new IllegalArgumentException("Game message is invalid (not one of the 4 colors)");
        }
    }
}
