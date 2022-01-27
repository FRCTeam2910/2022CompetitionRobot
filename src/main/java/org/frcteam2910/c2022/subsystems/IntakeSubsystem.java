package org.frcteam2910.c2022.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSubsystem implements Subsystem {
    private TalonFX motor = new TalonFX(0);
    private Solenoid extensionSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

    private double motorSpeed = 0.0;
    private boolean extended = true;

    public IntakeSubsystem() {}

    @Override
    public void periodic() {
        motor.set(TalonFXControlMode.PercentOutput, motorSpeed);
        extensionSolenoid.set(extended);
    }

    public void setFeederSpeed(double motorSpeed) {
        this.motorSpeed = motorSpeed;
    }

    public void setExtended(boolean extended) {
        this.extended = extended;
    }
}
