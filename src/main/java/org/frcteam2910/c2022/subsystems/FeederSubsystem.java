package org.frcteam2910.c2022.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2022.Constants;

public class FeederSubsystem implements Subsystem {
    private final static double FEEDER_GEAR_RATIO = 1.0 / 4.0;
    private final static double FEEDER_PULLEY_RADIUS = Units.inchesToMeters(0.5);

    private final static double BALL_TRAVEL_COEFFICIENT = 0.5;
    private final static double SENSOR_POSITION_COEFFICIENT = (1.0 / 2048.0) * FEEDER_GEAR_RATIO * 2.0
            * FEEDER_PULLEY_RADIUS * Math.PI * BALL_TRAVEL_COEFFICIENT;

    private final TalonFX motor = new TalonFX(Constants.FEEDER_MOTOR_PORT);
    private final DigitalInput fullSensor = new DigitalInput(Constants.FEEDER_SENSOR_FULL_PORT);
    private final DigitalInput entrySensor = new DigitalInput(Constants.FEEDER_SENSOR_ENTRY_PORT);
    // private Solenoid feederSolenoid = new Solenoid(PneumaticsModuleType.REVPH,
    // Constants.FEEDER_SOLENOID_PORT);

    private double motorSpeed;

    public FeederSubsystem() {
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        motor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        motor.setNeutralMode(NeutralMode.Brake);
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Feeder");
        shuffleboardTab.addBoolean("Full Sensor", this::isFull);
        shuffleboardTab.addBoolean("Entry Sensor", this::isBallAtEntry);
        // feederSolenoid.set(true);
    }

    @Override
    public void periodic() {
        motor.set(TalonFXControlMode.PercentOutput, motorSpeed);
    }

    public void setFeederSpeed(double motorSpeed) {
        this.motorSpeed = motorSpeed;
    }

    public double getFeederSpeed() {
        return motorSpeed;
    }

    public double getPosition() {
        return motor.getSelectedSensorPosition() * SENSOR_POSITION_COEFFICIENT;
    }

    public boolean isBallAtEntry() {
        return !entrySensor.get();
    }

    public boolean isFull() {
        return !fullSensor.get();
    }
}
