package org.frcteam2910.c2022.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2022.Constants;

public class FeederSubsystem implements Subsystem {
    private final TalonFX motor = new TalonFX(Constants.FEEDER_MOTOR_PORT);

    private double motorSpeed;

    public FeederSubsystem() {}

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
}
