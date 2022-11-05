package org.frcteam2910.c2022.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class SysIdLogger {
    protected static final int DATA_BUFFER_SIZE = 36000;

    private static final int THREAD_PRIORITY = 15;
    private static final int HAL_THREAD_PRIORITY = 40;

    protected double voltageCommand = 0.0;
    protected double motorVoltage = 0.0;
    protected double timestamp = 0.0;
    protected double startTime = 0.0;
    protected boolean rotate = false;
    protected String testType = "";
    protected String mechanism = "";
    protected final List<Double> data = new ArrayList<>(DATA_BUFFER_SIZE);

    public SysIdLogger() {
        System.out.println("Initializing logger");
        LiveWindow.disableAllTelemetry();
        SmartDashboard.putNumber("SysIdVoltageCommand", 0.0);
        SmartDashboard.putString("SysIdTestType", "");
        SmartDashboard.putString("SysIdTest", "");
        SmartDashboard.putBoolean("SysIdRotate", false);
        SmartDashboard.putBoolean("SysIdOverflow", false);
        SmartDashboard.putBoolean("SysIdWrongMech", false);
    }

    public void initLogging() {
        mechanism = SmartDashboard.getString("SysIdTest", "");

        SmartDashboard.putBoolean("SysIdWrongMech", isWrongMechanism());

        testType = SmartDashboard.getString("SysIdTestType", "");
        rotate = SmartDashboard.getBoolean("SysIdRotate", false);
        voltageCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0.0);
        startTime = Timer.getFPGATimestamp();
        data.clear();
    }

    public void sendData() {
        System.out.printf("Collected: %d data points.%n", data.size());

        SmartDashboard.putBoolean("SysIdOverflow", data.size() >= DATA_BUFFER_SIZE);

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < data.size(); ++i) {
            sb.append(data.get(i));
            if (i < data.size() - 1) {
                sb.append(',');
            }
        }

        SmartDashboard.putString("SysIdTelemetry", sb.toString());

        reset();
    }

    protected void updateData() {
        timestamp = Timer.getFPGATimestamp();

        // Don't let robot move if it's characterizing the wrong mechanism
        if (!isWrongMechanism()) {
            if (testType.equals("Quasistatic")) {
                motorVoltage = voltageCommand * (timestamp - startTime);
            } else if (testType.equals("Dynamic")) {
                motorVoltage = voltageCommand;
            } else {
                motorVoltage = 0.0;
            }
        } else {
            motorVoltage = 0.0;
        }
    }

    protected void reset() {
        motorVoltage = 0.0;
        timestamp = 0.0;
        startTime = 0.0;
        data.clear();
    }

    protected boolean isWrongMechanism() {
        return false;
    }
}
