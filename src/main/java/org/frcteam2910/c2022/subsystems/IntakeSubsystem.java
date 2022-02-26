package org.frcteam2910.c2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2022.Constants;

public class IntakeSubsystem implements Subsystem {
    private TalonFX motor = new TalonFX(Constants.INTAKE_LEFT_MOTOR_PORT);
    private Solenoid extensionSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_PORT);

    private double motorSpeed = 0.0;
    private boolean extended = true;

    public IntakeSubsystem() {
    }

    @Override
    public void periodic() {
        // motor.set(TalonFXControlMode.PercentOutput, motorSpeed);
        extensionSolenoid.set(extended);
    }

    public void setIntakeSpeed(double motorSpeed) {
        this.motorSpeed = motorSpeed;
    }

    public void setExtended(boolean extended) {
        this.extended = extended;
    }
}
