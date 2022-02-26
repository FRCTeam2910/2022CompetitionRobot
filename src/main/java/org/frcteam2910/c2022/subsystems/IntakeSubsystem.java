package org.frcteam2910.c2022.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2022.Constants;

public class IntakeSubsystem implements Subsystem {
    private TalonFX leftMotor = new TalonFX(Constants.INTAKE_LEFT_MOTOR_PORT);
    private TalonFX rightMotor = new TalonFX(Constants.INTAKE_RIGHT_MOTOR_PORT);
    private Solenoid extensionSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_PORT);

    private double motorSpeed = 0.0;
    private boolean extended = true;

    public IntakeSubsystem() {
        rightMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        rightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        leftMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        leftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        rightMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        leftMotor.set(TalonFXControlMode.PercentOutput, motorSpeed);
        rightMotor.set(TalonFXControlMode.PercentOutput, motorSpeed);
        extensionSolenoid.set(extended);
    }

    public void setIntakeSpeed(double motorSpeed) {
        this.motorSpeed = motorSpeed;
    }

    public void setExtended(boolean extended) {
        this.extended = extended;
    }
}
