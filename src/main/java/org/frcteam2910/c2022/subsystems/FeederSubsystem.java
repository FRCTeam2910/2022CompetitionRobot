package org.frcteam2910.c2022.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2022.Constants;

public class FeederSubsystem implements Subsystem {
    private final TalonFX motor = new TalonFX(Constants.FEEDER_MOTOR_PORT);
    private DigitalInput fullSensor = new DigitalInput(Constants.FEEDER_SENSOR_FULL_PORT);
    private DigitalInput entrySensor = new DigitalInput(Constants.FEEDER_SENSOR_ENTRY_PORT);
    private Solenoid feederSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.FEEDER_SOLENOID_PORT);

    private double motorSpeed;

    public FeederSubsystem() {
        feederSolenoid.set(true);
    }

    @Override
    public void periodic() {
        motor.set(TalonFXControlMode.PercentOutput, motorSpeed);
    }

    public void setFeederSpeed(double motorSpeed){
        this.motorSpeed = motorSpeed;
    }

    public double getFeederSpeed(){
        return motorSpeed;
    }

    public boolean getEntrySensor() { return entrySensor.get();}

    public boolean getFullSensor() { return fullSensor.get();}
}
