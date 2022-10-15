package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.google.errorprone.annotations.concurrent.GuardedBy;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.common.robot.UpdateManager;

import static org.frcteam2910.c2020.Constants.INTAKE_EXTENSION_SOLENOID;
import static org.frcteam2910.c2020.Constants.INTAKE_MOTOR;

public class IntakeSubsystem implements Subsystem, UpdateManager.Updatable {
    private TalonFX motor = new TalonFX(INTAKE_MOTOR);
    private Solenoid extensionSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, INTAKE_EXTENSION_SOLENOID);

    private final Object stateLock = new Object();
    @GuardedBy("stateLock")
    private double motorOutput = 0.0;
    @GuardedBy("stateLock")
    private boolean extended = false;

    private final NetworkTableEntry motorSpeedEntry;
    private final NetworkTableEntry isExtendedEntry;

    public IntakeSubsystem() {
        motor.setInverted(true);

        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        motorSpeedEntry = tab.add("Motor Speed", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
        isExtendedEntry = tab.add("Is Extended", false).withPosition(0, 1).withSize(1, 1).getEntry();
    }

    @Override
    public void update(double time, double dt) {
        double localMotorOutput;
        boolean localExtended;
        synchronized (stateLock) {
            localMotorOutput = motorOutput;
            localExtended = extended;
        }

        motor.set(ControlMode.PercentOutput, localMotorOutput);
        if (localExtended != extensionSolenoid.get()) {
            extensionSolenoid.set(localExtended);
        }
    }

    public boolean isExtended() {
        synchronized (stateLock) {
            return extended;
        }
    }

    public void setExtended(boolean extended) {
        synchronized (stateLock) {
            this.extended = extended;
        }
    }

    public void setMotorOutput(double motorOutput) {
        synchronized (stateLock) {
            this.motorOutput = motorOutput;
        }
    }

    public double getMotorOutput() {
        synchronized (stateLock) {
            return motor.getMotorOutputPercent();
        }
    }

    @Override
    public void periodic() {
        motorSpeedEntry.setDouble(getMotorOutput());
        isExtendedEntry.setBoolean(isExtended());
    }
}
