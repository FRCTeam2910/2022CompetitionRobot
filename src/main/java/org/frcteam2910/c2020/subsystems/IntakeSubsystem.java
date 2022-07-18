package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.google.errorprone.annotations.concurrent.GuardedBy;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.common.robot.UpdateManager;

public class IntakeSubsystem implements Subsystem, UpdateManager.Updatable {
    public static final double MIN_INTAKE_MOVEMENT_PRESSURE = 60.0;

    private TalonFX right_intake_motor = new TalonFX(Constants.INTAKE_MOTOR_PORT);
    private Solenoid topExtentionSolenoid = new Solenoid(PneumaticsModuleType.REVPH,
            Constants.TOP_INTAKE_EXTENSION_SOLENOID);
    private Solenoid bottomExtensionSolenoid = new Solenoid(PneumaticsModuleType.REVPH,
            Constants.BOTTOM_INTAKE_EXTENSION_SOLENOID);

    private final Object stateLock = new Object();
    @GuardedBy("stateLock")
    private double motorOutput = 0.0;
    @GuardedBy("stateLock")
    private boolean topExtended = false;
    @GuardedBy("stateLock")
    private boolean bottomExtended = false;
    @GuardedBy("stateLock")
    private boolean intakeCurrentThreshHoldPassed = false;

    private final NetworkTableEntry leftMotorSpeedEntry;
    private final NetworkTableEntry rightMotorSpeedEntry;
    private final NetworkTableEntry isTopExtendedEntry;
    private final NetworkTableEntry isBottomExtendedEntry;
    private final NetworkTableEntry leftMotorCurrent;

    private Timer fifthBallTimer;
    private boolean isTimerRunning;

    public IntakeSubsystem() {
        fifthBallTimer = new Timer();
        this.isTimerRunning = false;

        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        leftMotorSpeedEntry = tab.add("Left Motor Speed", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
        rightMotorSpeedEntry = tab.add("Right Motor Speed", 0.0).withPosition(1, 0).withSize(1, 1).getEntry();
        isTopExtendedEntry = tab.add("Is Top Extended", false).withPosition(0, 1).withSize(1, 1).getEntry();
        isBottomExtendedEntry = tab.add("Is Bottom Extended", false).withPosition(1, 1).withSize(1, 1).getEntry();
        leftMotorCurrent = tab.add("Left Motor Current", 0.0).withPosition(0, 2).withSize(1, 1).getEntry();
    }

    @Override
    public void update(double time, double dt) {
        double localMotorOutput;
        boolean localTopExtended;
        boolean localBottomExtended;
        synchronized (stateLock) {
            if (right_intake_motor.getStatorCurrent() > 20) {
                if (!isTimerRunning) {
                    fifthBallTimer.start();
                    isTimerRunning = true;
                }
            } else {
                fifthBallTimer.stop();
                fifthBallTimer.reset();
                isTimerRunning = false;
                intakeCurrentThreshHoldPassed = false;
            }

            if (fifthBallTimer.get() > 0.25) {
                intakeCurrentThreshHoldPassed = true;
            }

            localMotorOutput = motorOutput;
            localTopExtended = topExtended;
            localBottomExtended = bottomExtended;
        }

        right_intake_motor.set(ControlMode.PercentOutput, localMotorOutput);
        if (localTopExtended != topExtentionSolenoid.get()) {
            topExtentionSolenoid.set(localTopExtended);
        }

        bottomExtensionSolenoid.set(true);

    }

    public boolean isTopExtended() {
        synchronized (stateLock) {
            return topExtended;
        }
    }

    public void setTopExtended(boolean topExtended) {
        synchronized (stateLock) {
            this.topExtended = topExtended;
        }
    }

    public void setMotorOutput(double motorOutput) {
        synchronized (stateLock) {
            this.motorOutput = motorOutput;
        }
    }

    public double getLeftMotorOutput() {
        synchronized (stateLock) {
            return right_intake_motor.getMotorOutputPercent();
        }
    }

    public double getRightMotorOutput() {
        synchronized (stateLock) {
            return right_intake_motor.getMotorOutputPercent();
        }
    }

    public void setBottomExtended(boolean extended) {
        synchronized (stateLock) {
            bottomExtended = extended;
        }
    }

    public boolean isBottomExtended() {
        synchronized (stateLock) {
            return bottomExtended;
        }
    }

    public boolean hasIntakeMotorPassedCurrentThreshHold() {
        synchronized (stateLock) {
            return intakeCurrentThreshHoldPassed;
        }
    }

    public double getLeftMotorCurrent() {
        synchronized (stateLock) {
            return right_intake_motor.getStatorCurrent();
        }
    }

    @Override
    public void periodic() {
        leftMotorSpeedEntry.setDouble(getLeftMotorOutput());
        rightMotorSpeedEntry.setDouble(getRightMotorOutput());
        isTopExtendedEntry.setBoolean(topExtentionSolenoid.get());
        isBottomExtendedEntry.setBoolean(bottomExtensionSolenoid.get());
        leftMotorCurrent.setDouble(getLeftMotorCurrent());
    }
}
