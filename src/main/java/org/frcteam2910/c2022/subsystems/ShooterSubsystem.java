package org.frcteam2910.c2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2022.Constants;
import org.frcteam2910.c2022.Robot;
import org.frcteam2910.common.motion.MotionProfile;

public class ShooterSubsystem implements Subsystem {
    public static final double HOOD_MAX_ANGLE = Math.toRadians(85.2);
    public static final double HOOD_TRANSFER_ANGLE = Math.toRadians(80.0);
    public static final double HOOD_TRAVERSE_RETRACT_ANGLE = Math.toRadians(60.0);
    public static final double HOOD_TRAVERSE_EXTEND_ANGLE = Math.toRadians(45.0);
    public static final double HOOD_MIN_ANGLE = Math.toRadians(0.0);

    private static final double HOOD_MOMENT_OF_INERTIA = Units.lbsToKilograms(Units.inchesToMeters(450));
    private static final double HOOD_GEAR_REDUCTION = (14.0 / 54.0) * (18.0 / 38.0) * (20.0 / 36.0) * (10.0 / 220.0);
    private static final double FLYWHEEL_GEAR_REDUCTION = 1.0;
    private static final double FLYWHEEL_ALLOWABLE_ERROR = Units.rotationsPerMinuteToRadiansPerSecond(150);

    private static final DCMotor HOOD_MOTOR = DCMotor.getFalcon500(1);
    private static final double HOOD_VELOCITY_CONSTANT = 5.5657;
    private static final double HOOD_ACCELERATION_CONSTANT = 0.098378;
    private static final double HOOD_SENSOR_POSITION_COEFFICIENT = (HOOD_GEAR_REDUCTION / 2048.0) * 2 * Math.PI;
    private static final double HOOD_SENSOR_VELOCITY_COEFFICIENT = HOOD_SENSOR_POSITION_COEFFICIENT * 10.0;

    private static final double FLYWHEEL_VELOCITY_CONSTANT = 0.017941;
    private static final double FLYWHEEL_ACCELERATION_CONSTANT = 0.0030108;
    private static final double FLYWHEEL_SENSOR_POSITION_COEFFICIENT = (FLYWHEEL_GEAR_REDUCTION / 2048.0) * 2 * Math.PI;
    private static final double FLYWHEEL_SENSOR_VELOCITY_COEFFICIENT = FLYWHEEL_SENSOR_POSITION_COEFFICIENT * 10.0;

    private static final MotionProfile.Constraints MOTION_CONSTRAINTS = new MotionProfile.Constraints(
            Math.toRadians(100.0), Math.toDegrees(350));

    private final TalonFX hoodAngleMotor = new TalonFX(Constants.HOOD_MOTOR_PORT);
    private final LinearSystem<N2, N1, N1> hoodPlant = LinearSystemId
            .createSingleJointedArmSystem(DCMotor.getFalcon500(2), HOOD_MOMENT_OF_INERTIA, HOOD_GEAR_REDUCTION);
    private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(hoodPlant, HOOD_MOTOR,
            1.0 / HOOD_GEAR_REDUCTION, 0.0, 0.0, Math.PI, 0.0, false);

    private final FlywheelSim flywheel = new FlywheelSim(DCMotor.getFalcon500(2), 1.0, Units.inchesToMeters(6));
    private final TalonFX flywheelPrimaryMotor = new TalonFX(Constants.FLYWHEEL_PRIMARY_MOTOR_PORT);
    private final TalonFX flywheelSecondaryMotor = new TalonFX(Constants.FLYWHEEL_SECONDARY_MOTOR_PORT);

    private boolean hoodZeroed = false;

    private FlywheelMode flywheelMode = FlywheelMode.DISABLED;
    private double flywheelVoltage = 0.0;
    private double flywheelTargetVelocity = 0.0;

    private HoodMode hoodMode = HoodMode.DISABLED;
    private double hoodVoltage = 0.0;
    private double hoodTargetAngle = 0.0;

    public ShooterSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Shooter");
        shuffleboardTab.addNumber("Flywheel Speed",
                () -> Units.radiansPerSecondToRotationsPerMinute(getFlywheelVelocity()));
        shuffleboardTab.addNumber("Target Flywheel Speed",
                () -> Units.radiansPerSecondToRotationsPerMinute(getTargetFlywheelVelocity()));
        shuffleboardTab.addNumber("Hood Target Angle", () -> Math.toDegrees(getHoodTargetAngle()));
        shuffleboardTab.addNumber("Hood Angle", () -> Math.toDegrees(getHoodAngle()));
        shuffleboardTab.addNumber("Hood Velocity", () -> Math.toDegrees(getHoodVelocity()));
        shuffleboardTab.addNumber("Hood Voltage", () -> hoodVoltage);
        shuffleboardTab.addBoolean("Is Hood at Angle", this::isHoodAtTargetAngle);
        shuffleboardTab.addBoolean("Is Flywheel at Speed", this::isFlywheelAtTargetSpeed);

        TalonFXConfiguration flywheelConfiguration = new TalonFXConfiguration();
        flywheelConfiguration.supplyCurrLimit.currentLimit = 5.0;
        flywheelConfiguration.supplyCurrLimit.enable = false;
        flywheelConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor
                .toFeedbackDevice();
        flywheelConfiguration.slot0.kP = 0.5;
        flywheelConfiguration.slot0.kI = 0.0;
        flywheelConfiguration.slot0.kD = 0.0;

        flywheelPrimaryMotor.configAllSettings(flywheelConfiguration);
        flywheelSecondaryMotor.configAllSettings(flywheelConfiguration);

        flywheelPrimaryMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        flywheelPrimaryMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        flywheelPrimaryMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        flywheelSecondaryMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        flywheelSecondaryMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

        TalonFXConfiguration hoodConfiguration = new TalonFXConfiguration();
        hoodConfiguration.motionCruiseVelocity = MOTION_CONSTRAINTS.maxVelocity / HOOD_SENSOR_VELOCITY_COEFFICIENT;
        hoodConfiguration.motionAcceleration = MOTION_CONSTRAINTS.maxAcceleration / HOOD_SENSOR_VELOCITY_COEFFICIENT;
        hoodConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        hoodConfiguration.slot0.kP = 0.5;
        hoodConfiguration.slot0.kI = 0.0;
        hoodConfiguration.slot0.kD = 0.0;
        hoodConfiguration.supplyCurrLimit.currentLimit = 15.0;
        hoodConfiguration.supplyCurrLimit.enable = true;

        hoodAngleMotor.configAllSettings(hoodConfiguration);

        hoodAngleMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        hoodAngleMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        hoodAngleMotor.setNeutralMode(NeutralMode.Brake);

        flywheelPrimaryMotor.configVoltageCompSaturation(12.0);
        flywheelSecondaryMotor.configVoltageCompSaturation(12.0);
        flywheelPrimaryMotor.enableVoltageCompensation(true);
        flywheelSecondaryMotor.enableVoltageCompensation(true);

        flywheelPrimaryMotor.setNeutralMode(NeutralMode.Coast);
        flywheelSecondaryMotor.setNeutralMode(NeutralMode.Coast);
        flywheelPrimaryMotor.setInverted(true);
        flywheelSecondaryMotor.setInverted(true);

        flywheelSecondaryMotor.follow(flywheelPrimaryMotor);
    }

    public void setFlywheelVoltage(double flywheelVoltage) {
        this.flywheelVoltage = flywheelVoltage;
        this.flywheelMode = FlywheelMode.VOLTAGE;
    }

    public double getFlywheelVelocity() {
        if (Robot.isSimulation()) {
            return flywheel.getAngularVelocityRadPerSec();
        } else {
            return flywheelPrimaryMotor.getSelectedSensorVelocity() * FLYWHEEL_SENSOR_VELOCITY_COEFFICIENT;
        }
    }

    public void setTargetFlywheelVelocity(double flywheelTargetVelocity) {
        this.flywheelTargetVelocity = flywheelTargetVelocity;
        this.flywheelMode = FlywheelMode.VELOCITY;
    }

    public double getTargetFlywheelVelocity() {
        return flywheelTargetVelocity;
    }

    public boolean isFlywheelAtTargetSpeed() {
        return Math.abs(getFlywheelVelocity() - getTargetFlywheelVelocity()) < FLYWHEEL_ALLOWABLE_ERROR;
    }

    public double getHoodAngle() {
        if (Robot.isSimulation()) {
            return hoodSim.getAngleRads();
        } else {
            return hoodAngleMotor.getSelectedSensorPosition() * HOOD_SENSOR_POSITION_COEFFICIENT;
        }
    }

    public double getHoodVelocity() {
        if (Robot.isSimulation()) {
            return hoodSim.getVelocityRadPerSec();
        } else {
            return hoodAngleMotor.getSelectedSensorVelocity() * HOOD_SENSOR_VELOCITY_COEFFICIENT;
        }
    }

    public void setHoodTargetAngle(double hoodTargetAngle) {
        this.hoodTargetAngle = hoodTargetAngle;
        this.hoodMode = HoodMode.POSITION;
    }

    public double getHoodTargetAngle() {
        return hoodTargetAngle;
    }

    public void setHoodVoltage(double hoodVoltage) {
        this.hoodVoltage = hoodVoltage;
        this.hoodMode = HoodMode.VOLTAGE;
    }

    public void setHoodZeroed(boolean zeroed) {
        this.hoodZeroed = zeroed;
    }

    public boolean isHoodZeroed() {
        return hoodZeroed;
    }

    public boolean isHoodAtTargetAngle() {
        return Math.abs(getHoodTargetAngle() - getHoodAngle()) < Constants.HOOD_ALLOWABLE_ERROR;
    }

    public void resetHoodAngle(double angle) {
        hoodAngleMotor.setSelectedSensorPosition(angle / HOOD_SENSOR_POSITION_COEFFICIENT);
    }

    public void simulationPeriodic() {
        flywheel.setInputVoltage(flywheelVoltage);
        flywheel.update(0.02);

        hoodSim.setInputVoltage(hoodVoltage);
        hoodSim.update(0.02);
    }

    public void enableCurrentLimits(boolean enabled) {
        SupplyCurrentLimitConfiguration configuration = new SupplyCurrentLimitConfiguration();
        configuration.currentLimit = 5.0;
        configuration.enable = enabled;
        flywheelPrimaryMotor.configSupplyCurrentLimit(configuration);
        flywheelSecondaryMotor.configSupplyCurrentLimit(configuration);
    }

    public void setHoodBrakeMode(boolean brake) {
        if (brake) {
            hoodAngleMotor.setNeutralMode(NeutralMode.Brake);
        } else {
            hoodAngleMotor.setNeutralMode(NeutralMode.Coast);
        }
    }

    @Override
    public void periodic() {
        switch (flywheelMode) {
            case DISABLED :
                flywheelPrimaryMotor.set(TalonFXControlMode.Disabled, 0.0);
                break;
            case VELOCITY :
                flywheelPrimaryMotor.set(TalonFXControlMode.Velocity,
                        flywheelTargetVelocity / FLYWHEEL_SENSOR_VELOCITY_COEFFICIENT, DemandType.ArbitraryFeedForward,
                        FLYWHEEL_VELOCITY_CONSTANT * flywheelTargetVelocity / 12.0);
                break;
            case VOLTAGE :
                flywheelPrimaryMotor.set(TalonFXControlMode.PercentOutput, flywheelVoltage / 12.0);
        }

        switch (hoodMode) {
            case DISABLED :
                hoodAngleMotor.set(TalonFXControlMode.Disabled, 0.0);
                break;
            case POSITION :
                if (isHoodZeroed()) {
                    hoodAngleMotor.set(TalonFXControlMode.MotionMagic,
                            hoodTargetAngle / HOOD_SENSOR_POSITION_COEFFICIENT);
                } else {
                    DriverStation.reportWarning("Attempted to set hood angle when it was not zeroed", false);
                }
                break;
            case VOLTAGE :
                hoodAngleMotor.set(TalonFXControlMode.PercentOutput, hoodVoltage / 12);
                break;
        }
    }

    private enum FlywheelMode {
        DISABLED, VELOCITY, VOLTAGE
    }

    private enum HoodMode {
        DISABLED, POSITION, VOLTAGE
    }
}