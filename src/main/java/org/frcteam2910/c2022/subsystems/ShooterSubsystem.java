package org.frcteam2910.c2022.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2022.Constants;
import org.frcteam2910.c2022.Robot;

import java.util.OptionalDouble;


public class ShooterSubsystem implements Subsystem {
    private static final double HOOD_MOMENT_OF_INERTIA = Units.lbsToKilograms(Units.inchesToMeters(450));
    private static final double HOOD_GEAR_REDUCTION = 85.0;
    private static final double FLYWHEEL_ALLOWABLE_ERROR = 0.1;
    private final FlywheelSim flywheel = new FlywheelSim(DCMotor.getFalcon500(2), 1.0, Units.inchesToMeters(6));
    private final LinearSystem hoodPlant = LinearSystemId.createSingleJointedArmSystem(DCMotor.getFalcon500(2), HOOD_MOMENT_OF_INERTIA, HOOD_GEAR_REDUCTION);
    private final LinearSystemSim hoodSim = new LinearSystemSim(hoodPlant);
    private final TalonFX flywheelPrimaryMotor = new TalonFX(Constants.FLYWHEEL_PRIMARY_MOTOR_PORT);
    private final TalonFX flywheelSecondaryMotor = new TalonFX(Constants.FLYWHEEL_SECONDARY_MOTOR_PORT);
    private double flywheelVoltage;
    private double hoodVoltage;
    private boolean isHoodZeroed;
    private double targetFlywheelSpeed;
    private double hoodTargetPosition = Double.NaN;
    private PIDController flywheelVelocityController = new PIDController(0.5, 0.0, 0.0);

    private final TalonFX hoodAngleMotor = new TalonFX(Constants.HOOD_MOTOR_PORT);

    public ShooterSubsystem(){
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Shooter");
        shuffleboardTab.addNumber("Flywheel Speed", () -> Units.radiansPerSecondToRotationsPerMinute(getFlywheelSpeed()));
        shuffleboardTab.addNumber("Hood Angle", () -> Math.toDegrees(getHoodAngle()));

        flywheelPrimaryMotor.configVoltageCompSaturation(12.0);
        flywheelSecondaryMotor.configVoltageCompSaturation(12.0);
        flywheelPrimaryMotor.enableVoltageCompensation(true);
        flywheelSecondaryMotor.enableVoltageCompensation(true);
    }

    public double getFlywheelSpeed(){
        return flywheel.getAngularVelocityRadPerSec();
    }

    public double getHoodAngle(){
        return hoodSim.getOutput(0);
    }

    public void setFlywheelVoltage(double flywheelVoltage) {
        this.flywheelVoltage = flywheelVoltage;
    }

    public void setHoodVoltage(double hoodVoltage){
        this.hoodVoltage = hoodVoltage;
    }

    public double getHoodVoltage() {
        return hoodVoltage;
    }

    public void setHoodZeroed(boolean zeroed) {
        this.isHoodZeroed = zeroed;
    }

    public void setHoodTargetPosition(double position){
        this.hoodTargetPosition = position;
    }

    public boolean isHoodAtTargetAngle() {
        return Math.abs(hoodTargetPosition - hoodSim.getOutput(0)) < Constants.HOOD_ALLOWABLE_ERROR;
    }

    public void setTargetFlywheelSpeed(double targetFlywheelSpeed){
        this.targetFlywheelSpeed = targetFlywheelSpeed;
    }

    public double getTargetFlywheelSpeed() {
        return targetFlywheelSpeed;
    }

    public double getFlywheelVelocity() {
        if(Robot.isSimulation()) {
            return flywheel.getAngularVelocityRadPerSec();
        } else {
            return flywheelPrimaryMotor.getActiveTrajectoryVelocity();
        }
    }

    public boolean isFlywheelAtTargetSpeed(){
        return Math.abs(getFlywheelVelocity() - targetFlywheelSpeed) < FLYWHEEL_ALLOWABLE_ERROR;
    }

    public OptionalDouble getHoodTargetPosition() {
        if(Double.isFinite(hoodTargetPosition)) {
            return OptionalDouble.of(hoodTargetPosition);
        } else {
            return OptionalDouble.empty();
        }
    }

    public boolean isHoodZeroed() {
        return isHoodZeroed;
    }

    public void zeroHoodMotor() {
        hoodAngleMotor.setSelectedSensorPosition(0.0);
    }

    public void simulationPeriodic() {
            flywheel.setInputVoltage(flywheelVoltage);
            flywheel.update(0.02);

            hoodSim.setInput(hoodVoltage);
            hoodSim.update(0.02);
    }

    @Override
    public void periodic() {
        if(!getHoodTargetPosition().isEmpty()){
            double targetAngle = getHoodTargetPosition().getAsDouble();
            hoodAngleMotor.set(TalonFXControlMode.Position, angleToTalonUnits(targetAngle));
        }
        flywheelVoltage = flywheelVelocityController.calculate(getFlywheelVelocity(), targetFlywheelSpeed);

        flywheelPrimaryMotor.set(TalonFXControlMode.PercentOutput, flywheelVoltage / 12.0);
        flywheelSecondaryMotor.set(TalonFXControlMode.PercentOutput, flywheelVoltage / 12.0);
    }

    private double angleToTalonUnits(double angle) {
        return angle * 2048 / (2 * Math.PI) * Constants.HOOD_MOTOR_TO_HOOD_GEAR_RATIO;
    }
}