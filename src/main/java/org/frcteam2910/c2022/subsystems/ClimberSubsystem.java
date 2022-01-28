package org.frcteam2910.c2022.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2022.Constants;

public class ClimberSubsystem implements Subsystem {
    private final DCMotor gearbox = DCMotor.getFalcon500(2);
    private final ElevatorSim climber = new ElevatorSim(gearbox, 22.0, Units.lbsToKilograms(60.0), Units.inchesToMeters(1), 0.1, 1.1);
    private final PWMTalonSRX motor = new PWMTalonSRX(Constants.CLIMBER_MOTOR_PORT);
    private final PIDController positionPID = new PIDController(10.0, 0.0, 0.0);
    private final PIDController velocityPID = new PIDController(3.0, 0.0, 0.0);

    private final Mechanism2d mech2d = new Mechanism2d(100, 120);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 10);
    private final MechanismLigament2d position = mech2dRoot.append(new MechanismLigament2d("Position", 0, 90));
    private final MechanismLigament2d motorOutput = mech2dRoot.append(new MechanismLigament2d("Motor Output", 0, 45, 10, new Color8Bit(150, 0, 255)));

    private double motorSpeed = 0.0;
    private double targetHeight = 0.0;
    private double targetVelocity = 0.0;
    private double voltage = 0.0;
    private boolean manual = true;
    private boolean positionControl = true;

    public static final double MAX_VELOCITY = 0.75;

    public ClimberSubsystem() {
        SmartDashboard.putData("Elevator Sim", mech2d);
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Climber");
        shuffleboardTab.addNumber("velocity", climber::getVelocityMetersPerSecond);
        shuffleboardTab.addNumber("target velocity", () -> targetVelocity);
        shuffleboardTab.addNumber("height", climber::getPositionMeters);
        shuffleboardTab.addNumber("target height", () -> targetHeight);
        shuffleboardTab.addNumber("voltage", () -> voltage);
    }

    public void periodic(){
        if (manual) {
            if(positionControl) {
                voltage = positionPID.calculate(climber.getPositionMeters(), motorSpeed) * 12;
            } else {
                voltage = targetVelocity / MAX_VELOCITY * 12; // Feedforward
                voltage *= 0.92; // Friction
                voltage += velocityPID.calculate(climber.getVelocityMetersPerSecond(), targetVelocity);
            }
        } else {
            voltage = positionPID.calculate(climber.getPositionMeters(), targetHeight) * 12;
        }
        climber.setInputVoltage(voltage);
        motor.setVoltage(voltage);
        climber.update(0.020);
        position.setLength(climber.getPositionMeters() * 100);
        motorOutput.setLength((motorSpeed + 1) * 50);
    }

    public void setMotorSpeed(double motorSpeed) {
        this.motorSpeed = motorSpeed;
    }

    public double getClimberPosition() {
        return climber.getPositionMeters();
    }
    
    public void setTargetHeight(double targetHeight) {
        this.targetHeight = targetHeight;
    }

    public double getTargetHeight() {
        return targetHeight;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void setPositionControl(boolean positionControl) {
        this.positionControl = positionControl;
        manual = true;
    }

    public boolean getPositionControl() {
        return positionControl;
    }

    public void setManual(boolean manual) {
        this.manual = manual;
    }

    public boolean getManual() {
        return manual;
    }

    public boolean isAtTargetHeight() {
        return Math.abs(climber.getPositionMeters() - targetHeight) < 0.01;
    }

    public PIDController getPID() { return positionPID; }
}
