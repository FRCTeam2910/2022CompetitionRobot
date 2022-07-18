package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;

public class ClimberSubsystem implements Subsystem {
    public static final double FORWARD_LIMIT = 400000.0;
    public static final double REVERSE_LIMIT = 60000.0;
    public static final boolean ENABLE_LIMITS = true;

    private final Solenoid lockSolenoid = new Solenoid(PneumaticsModuleType.REVPH,
            Constants.CLIMBER_LOCK_SOLENOID_PORT);
    private final TalonFX motor = new TalonFX(Constants.CLIMBER_MOTOR_PORT);

    public ClimberSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.supplyCurrLimit.enable = false;
        config.forwardSoftLimitEnable = ENABLE_LIMITS;
        config.forwardSoftLimitThreshold = FORWARD_LIMIT;
        config.reverseSoftLimitEnable = ENABLE_LIMITS;
        config.reverseSoftLimitThreshold = REVERSE_LIMIT;

        motor.configAllSettings(config);
        motor.setNeutralMode(NeutralMode.Brake);

        ShuffleboardTab tab = Shuffleboard.getTab("Climber");
        tab.addNumber("Height", this::getHeight);
    }

    public void unlock() {
        lockSolenoid.set(true);
    }

    public void lock() {
        lockSolenoid.set(false);
    }

    public void setMotorOutput(double output) {
        motor.set(TalonFXControlMode.PercentOutput, output);
    }

    public double getClimberVelocity() {
        return motor.getSensorCollection().getIntegratedSensorVelocity();
    }

    public void zeroHeight() {
        motor.setSelectedSensorPosition(0.0);
    }

    public double getHeight() {
        return motor.getSelectedSensorPosition();
    }

    public void enableSoftLimits() {
        motor.overrideSoftLimitsEnable(true);
    }

    public void disableSoftLimits() {
        motor.overrideSoftLimitsEnable(false);
    }
}
