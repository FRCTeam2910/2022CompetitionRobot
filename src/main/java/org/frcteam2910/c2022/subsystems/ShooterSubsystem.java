package org.frcteam2910.c2022.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2022.Constants;
import org.frcteam2910.c2022.Robot;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.motion.MotionProfile;

public class ShooterSubsystem implements Subsystem {
    public static final double HOOD_MAX_ANGLE = Math.toRadians(85.2);
    public static final double HOOD_PREPARE_TRANSFER_ANGLE = Math.toRadians(84.2);
    public static final double HOOD_TRANSFER_ANGLE = Math.toRadians(76.4);
    public static final double HOOD_TRAVERSE_RETRACT_ANGLE = Math.toRadians(45.0);
    public static final double HOOD_TRAVERSE_EXTEND_ANGLE_HIGH = Math.toRadians(20.0);
    public static final double HOOD_TRAVERSE_EXTEND_ANGLE_TRAVERSE = Math.toRadians(25.0);
    public static final double HOOD_MIN_ANGLE = Math.toRadians(0.0);

    private static final double HOOD_MOMENT_OF_INERTIA = Units.lbsToKilograms(Units.inchesToMeters(450));
    private static final double HOOD_GEAR_REDUCTION = (14.0 / 54.0) * (18.0 / 38.0) * (38.0 / 18.0) * (10.0 / 220.0);
    private static final double FLYWHEEL_GEAR_REDUCTION = 36.0 / 53.0;
    private static final double FLYWHEEL_ALLOWABLE_ERROR = Units.rotationsPerMinuteToRadiansPerSecond(75);

    private static final DCMotor HOOD_MOTOR = DCMotor.getFalcon500(1);
    private static final double HOOD_VELOCITY_CONSTANT = 5.5657;
    private static final double HOOD_ACCELERATION_CONSTANT = 0.098378;
    private static final double HOOD_SENSOR_POSITION_COEFFICIENT = (HOOD_GEAR_REDUCTION / 2048.0) * 2 * Math.PI;
    private static final double HOOD_SENSOR_VELOCITY_COEFFICIENT = HOOD_SENSOR_POSITION_COEFFICIENT * 10.0;

    private static final double FLYWHEEL_VELOCITY_CONSTANT = 0.028;
    private static final double FLYWHEEL_ACCELERATION_CONSTANT = 0.0030108;
    private static final double FLYWHEEL_SENSOR_POSITION_COEFFICIENT = (FLYWHEEL_GEAR_REDUCTION / 2048.0) * 2 * Math.PI;
    private static final double FLYWHEEL_SENSOR_VELOCITY_COEFFICIENT = FLYWHEEL_SENSOR_POSITION_COEFFICIENT * 10.0;
    public static final double FLYWHEEL_IDLE_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(2000);

    private static final MotionProfile.Constraints FAST_MOTION_CONSTRAINTS = new MotionProfile.Constraints(
            Math.toRadians(450.0), Math.toRadians(2000.0));
    private static final MotionProfile.Constraints SLOW_MOTION_CONSTRAINTS = new MotionProfile.Constraints(
            Math.toRadians(450.0), Math.toRadians(500.0));

    private final TalonFX hoodAngleMotor = new TalonFX(Constants.HOOD_MOTOR_PORT);
    private final LinearSystem<N2, N1, N1> hoodPlant = LinearSystemId
            .createSingleJointedArmSystem(DCMotor.getFalcon500(2), HOOD_MOMENT_OF_INERTIA, HOOD_GEAR_REDUCTION);
    private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(hoodPlant, HOOD_MOTOR,
            1.0 / HOOD_GEAR_REDUCTION, 0.0, 0.0, Math.PI, 0.0, false);

    private final FlywheelSim flywheel = new FlywheelSim(DCMotor.getFalcon500(2), 1.0, Units.inchesToMeters(6));
    private final TalonFX flywheelPrimaryMotor = new TalonFX(Constants.FLYWHEEL_PRIMARY_MOTOR_PORT);
    private final TalonFX flywheelSecondaryMotor = new TalonFX(Constants.FLYWHEEL_SECONDARY_MOTOR_PORT);

    private double flywheelVoltage;
    private double hoodVoltage;
    private double hoodTargetAngle = Math.toRadians(5.0);
    private boolean isHoodZeroed = false;
    private boolean flywheelDisabled = false;
    private double targetFlywheelSpeed;
    private double shootingOffset = 0.0;
    private double angleOffset = 0.0;
    private final NetworkTableEntry shooterRPMOffsetEntry;
    private final NetworkTableEntry hoodAngleOffsetEntry;

    public ShooterSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Shooter");
        shuffleboardTab.addNumber("Flywheel Speed",
                () -> Units.radiansPerSecondToRotationsPerMinute(getFlywheelVelocity()));
        shuffleboardTab.addNumber("Target Flywheel Speed",
                () -> Units.radiansPerSecondToRotationsPerMinute(getTargetFlywheelSpeed()));
        shuffleboardTab.addNumber("Hood Target Angle", () -> Math.toDegrees(getHoodTargetPosition()));
        shuffleboardTab.addNumber("Hood Angle", () -> Math.toDegrees(getHoodAngle()));
        shuffleboardTab.addNumber("Hood Velocity", () -> Math.toDegrees(getHoodVelocity()));
        shuffleboardTab.addNumber("Hood Voltage", () -> hoodVoltage);
        shuffleboardTab.addBoolean("Is Hood at Angle", this::isHoodAtTargetAngle);
        shuffleboardTab.addBoolean("Is Flywheel at Speed", this::isFlywheelAtTargetSpeed);
        shooterRPMOffsetEntry = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME)
                .add(Constants.SHOOTER_OFFSET_ENTRY_NAME, 0.0).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -250.0, "max", 250.0, "Block increment", 25.0)).withPosition(2, 1)
                .getEntry();
        hoodAngleOffsetEntry = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME)
                .add(Constants.HOOD_OFFSET_ENTRY_NAME, 0.0).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -10.0, "max", 10.0, "Block increment", 0.5)).withPosition(2, 2)
                .getEntry();

        TalonFXConfiguration flywheelConfiguration = new TalonFXConfiguration();
        flywheelConfiguration.supplyCurrLimit.currentLimit = 20.0;
        flywheelConfiguration.supplyCurrLimit.enable = false;
        flywheelConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor
                .toFeedbackDevice();
        flywheelConfiguration.slot0.kP = 0.25;
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
        hoodConfiguration.motionCruiseVelocity = FAST_MOTION_CONSTRAINTS.maxVelocity / HOOD_SENSOR_VELOCITY_COEFFICIENT;
        hoodConfiguration.motionAcceleration = FAST_MOTION_CONSTRAINTS.maxAcceleration
                / HOOD_SENSOR_VELOCITY_COEFFICIENT;
        hoodConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        hoodConfiguration.slot0.kP = 0.5;
        hoodConfiguration.slot0.kI = 0.0;
        hoodConfiguration.slot0.kD = 0.0;
        hoodConfiguration.supplyCurrLimit.currentLimit = 50.0;
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
    }

    public double getHoodAngle() {
        if (Robot.isSimulation()) {
            return hoodSim.getAngleRads();
        } else {
            return hoodAngleMotor.getSelectedSensorPosition() * HOOD_SENSOR_POSITION_COEFFICIENT;
        }
    }

    public void setFlywheelVoltage(double flywheelVoltage) {
        this.flywheelVoltage = flywheelVoltage;
    }

    public void setHoodVoltage(double hoodVoltage) {
        this.hoodVoltage = hoodVoltage;
    }

    public double getHoodVelocity() {
        if (Robot.isSimulation()) {
            return hoodSim.getVelocityRadPerSec();
        } else {
            return hoodAngleMotor.getSelectedSensorVelocity() * HOOD_SENSOR_VELOCITY_COEFFICIENT;
        }
    }

    public void setHoodZeroed(boolean zeroed) {
        this.isHoodZeroed = zeroed;
    }

    public boolean isHoodAtTargetAngle() {
        return isHoodAtTargetAngle(false);
    }

    public boolean isHoodAtTargetAngle(boolean climbing) {
        if (Robot.isSimulation()) {
            return Math.abs(getHoodTargetPosition() - hoodSim.getAngleRads()) < Constants.HOOD_SHOOTING_ALLOWABLE_ERROR;
        } else if (climbing) {
            return Math.abs(getHoodTargetPosition() - getHoodAngle()) < Constants.HOOD_CLIMBING_ALLOWABLE_ERROR;
        } else {
            return Math.abs(getHoodTargetPosition() - getHoodAngle()) < Constants.HOOD_SHOOTING_ALLOWABLE_ERROR;
        }
    }

    public void setTargetFlywheelSpeed(double targetFlywheelSpeed) {
        this.targetFlywheelSpeed = targetFlywheelSpeed + Units.rotationsPerMinuteToRadiansPerSecond(shootingOffset);
    }

    public double getTargetFlywheelSpeed() {
        return targetFlywheelSpeed;
    }

    public double getFlywheelVelocity() {
        if (Robot.isSimulation()) {
            return flywheel.getAngularVelocityRadPerSec();
        } else {
            return flywheelPrimaryMotor.getSelectedSensorVelocity() * FLYWHEEL_SENSOR_VELOCITY_COEFFICIENT;
        }
    }

    public boolean isFlywheelAtTargetSpeed() {
        return Math.abs(getFlywheelVelocity() - targetFlywheelSpeed) < FLYWHEEL_ALLOWABLE_ERROR;
    }

    public void setHoodTargetPosition(double position) {
        setHoodTargetPosition(position, false);
    }

    public void setHoodTargetPosition(double position, boolean offset) {
        if (!offset) {
            hoodTargetAngle = MathUtils.clamp(position, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
        } else {
            hoodTargetAngle = MathUtils.clamp(position + Units.degreesToRadians(angleOffset), HOOD_MIN_ANGLE,
                    HOOD_MAX_ANGLE);
        }
    }

    public double getHoodTargetPosition() {
        return hoodTargetAngle;
    }

    public boolean isHoodZeroed() {
        return isHoodZeroed;
    }

    public void setHoodMotorSensorPosition(double position) {
        hoodAngleMotor.setSelectedSensorPosition(position / HOOD_SENSOR_POSITION_COEFFICIENT);
    }

    public void simulationPeriodic() {
        flywheel.setInputVoltage(flywheelVoltage);
        flywheel.update(0.02);

        hoodSim.setInputVoltage(hoodVoltage);
        hoodSim.update(0.02);
    }

    public void enableCurrentLimits(boolean enabled) {
        SupplyCurrentLimitConfiguration configuration = new SupplyCurrentLimitConfiguration();
        configuration.currentLimit = 20.0;
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

    public void setFastHoodConfig(boolean fast) {
        if (fast) {
            hoodAngleMotor
                    .configMotionCruiseVelocity(FAST_MOTION_CONSTRAINTS.maxVelocity / HOOD_SENSOR_VELOCITY_COEFFICIENT);
            hoodAngleMotor.configMotionAcceleration(
                    FAST_MOTION_CONSTRAINTS.maxAcceleration / HOOD_SENSOR_VELOCITY_COEFFICIENT);
        } else {
            hoodAngleMotor
                    .configMotionCruiseVelocity(SLOW_MOTION_CONSTRAINTS.maxVelocity / HOOD_SENSOR_VELOCITY_COEFFICIENT);
            hoodAngleMotor.configMotionAcceleration(
                    SLOW_MOTION_CONSTRAINTS.maxAcceleration / HOOD_SENSOR_VELOCITY_COEFFICIENT);
        }
    }

    public void disableFlywheel() {
        flywheelDisabled = true;
    }

    @Override
    public void periodic() {
        if (isHoodZeroed) {
            hoodAngleMotor.set(TalonFXControlMode.MotionMagic, hoodTargetAngle / HOOD_SENSOR_POSITION_COEFFICIENT);
        } else {
            hoodAngleMotor.set(TalonFXControlMode.PercentOutput, hoodVoltage / 12);
        }

        if (flywheelDisabled) {
            flywheelPrimaryMotor.set(TalonFXControlMode.PercentOutput, 0.0);
            flywheelSecondaryMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        } else {
            double feedForward = FLYWHEEL_VELOCITY_CONSTANT * targetFlywheelSpeed / 12.0;
            flywheelPrimaryMotor.set(TalonFXControlMode.Velocity,
                    targetFlywheelSpeed / FLYWHEEL_SENSOR_VELOCITY_COEFFICIENT, DemandType.ArbitraryFeedForward,
                    feedForward);
            flywheelSecondaryMotor.set(TalonFXControlMode.Velocity,
                    targetFlywheelSpeed / FLYWHEEL_SENSOR_VELOCITY_COEFFICIENT, DemandType.ArbitraryFeedForward,
                    feedForward);
        }

        shootingOffset = shooterRPMOffsetEntry.getDouble(0.0);
        angleOffset = hoodAngleOffsetEntry.getDouble(0.0);
    }
}