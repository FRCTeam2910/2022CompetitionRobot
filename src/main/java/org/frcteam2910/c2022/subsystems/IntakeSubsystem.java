package org.frcteam2910.c2022.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2022.Constants;

public class IntakeSubsystem implements Subsystem {
    private final TalonFX leftMotor = new TalonFX(Constants.INTAKE_LEFT_MOTOR_PORT);
    private final TalonFX rightMotor = new TalonFX(Constants.INTAKE_RIGHT_MOTOR_PORT);
    private final Solenoid extensionSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_PORT);

    private double motorSpeed = 0.0;
    private boolean extended = false;

    public IntakeSubsystem() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.supplyCurrLimit.currentLimit = 30;
        configuration.supplyCurrLimit.enable = true;

        rightMotor.configAllSettings(configuration);
        leftMotor.configAllSettings(configuration);

        rightMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        rightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        leftMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        leftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        rightMotor.setInverted(true);
    }

    public void setIntakeSpeed(double motorSpeed) {
        this.motorSpeed = motorSpeed;
    }

    public void setExtended(boolean extended) {
        this.extended = extended;
    }

    @Override
    public void periodic() {
        leftMotor.set(TalonFXControlMode.PercentOutput, motorSpeed);
        rightMotor.set(TalonFXControlMode.PercentOutput, motorSpeed);
        extensionSolenoid.set(extended);
    }
}
