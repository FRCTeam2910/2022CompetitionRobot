package org.frcteam2910.c2022.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2022.Constants;
import org.frcteam2910.c2022.Robot;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.motion.MotionProfile;

public class ClimberSubsystem implements Subsystem {
    public static final double HEIGHT_CHANGE = 6.75;
    public static final double MAX_HEIGHT = Units.inchesToMeters(32.75);
    public static final double MID_RUNG_HEIGHT = Units.inchesToMeters(30.0);
    public static final double TRAVERSE_EXTEND_HEIGHT = Units.inchesToMeters(28.5);
    public static final double TRAVERSE_RUNG_PARTWAY_HEIGHT = Units.inchesToMeters(20.0);
    public static final double TRAVERSE_RUNG_HEIGHT = Units.inchesToMeters(18.25);
    public static final double HOOD_PASSAGE_HEIGHT = Units.inchesToMeters(3.5);
    public static final double HOOD_TRANSFER_HEIGHT = Units.inchesToMeters(-0.5);
    public static final double MIN_HEIGHT = Units.feetToMeters(0.0);

    private static final DCMotor MOTOR = DCMotor.getFalcon500(2);
    private static final double REDUCTION = (16.0 / 36.0) * (28.0 / 36.0) * (18.0 / 48.0);
    private static final double MASS = Units.lbsToKilograms(60.0);
    private static final double RADIUS = Units.inchesToMeters(0.90);

    private static final double VELOCITY_CONSTANT = 1.0 / (RADIUS * REDUCTION * MOTOR.KvRadPerSecPerVolt);
    private static final double ACCELERATION_CONSTANT = (MOTOR.rOhms * RADIUS * MASS * REDUCTION) / (MOTOR.KtNMPerAmp);

    private static final double ALLOWABLE_POSITION_ERROR = Units.inchesToMeters(0.5);

    private static final double SENSOR_POSITION_COEFFICIENT = REDUCTION * RADIUS * 2 * Math.PI / 2048.0;
    private static final double SENSOR_VELOCITY_COEFFICIENT = SENSOR_POSITION_COEFFICIENT * 10.0;

    private static final MotionProfile.Constraints FAST_MOTION_CONSTRAINTS = new MotionProfile.Constraints(
            Units.feetToMeters(6.0), Units.feetToMeters(12.0));
    private static final MotionProfile.Constraints SLOW_MOTION_CONSTRAINTS = new MotionProfile.Constraints(
            Units.feetToMeters(3.0), Units.feetToMeters(6.0));

    private final LinearSystem<N2, N1, N1> plant = LinearSystemId.identifyPositionSystem(VELOCITY_CONSTANT,
            ACCELERATION_CONSTANT);
    private final ElevatorSim simulation = new ElevatorSim(plant, MOTOR, 1.0 / REDUCTION, RADIUS, 0.0,
            MAX_HEIGHT * 1.1);
    private final TalonFX firstLeftMotor = new TalonFX(Constants.CLIMBER_FIRST_LEFT_MOTOR_PORT, "CANivore");
    private final TalonFX firstRightMotor = new TalonFX(Constants.CLIMBER_FIRST_RIGHT_MOTOR_PORT, "CANivore");
    private final TalonFX secondLeftMotor = new TalonFX(Constants.CLIMBER_SECOND_LEFT_MOTOR_PORT, "CANivore");
    private final TalonFX secondRightMotor = new TalonFX(Constants.CLIMBER_SECOND_RIGHT_MOTOR_PORT, "CANivore");

    private Mode mode = Mode.VOLTAGE;
    private boolean zeroed = false;
    private double targetHeight = 0.0;
    private double targetVoltage = 0.0;

    public ClimberSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Climber");
        shuffleboardTab.addNumber("Target Height", () -> Units.metersToInches(getTargetHeight()));
        shuffleboardTab.addNumber("Height", () -> Units.metersToInches(getCurrentHeight()));
        shuffleboardTab.addNumber("Velocity", () -> Units.metersToInches(getCurrentVelocity()));
        shuffleboardTab.addNumber("Voltage", () -> targetVoltage);
        shuffleboardTab.addString("Mode", () -> mode.name());

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.motionCruiseVelocity = FAST_MOTION_CONSTRAINTS.maxVelocity / SENSOR_VELOCITY_COEFFICIENT;
        configuration.motionAcceleration = FAST_MOTION_CONSTRAINTS.maxAcceleration / SENSOR_VELOCITY_COEFFICIENT;
        configuration.slot0.kP = 0.25;
        configuration.slot0.kI = 0.0;
        configuration.slot0.kD = 0.0;
        configuration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        // configuration.supplyCurrLimit.currentLimit = 100.0;
        // configuration.supplyCurrLimit.enable = true;
        configuration.voltageCompSaturation = 12.0;

        firstLeftMotor.configAllSettings(configuration);
        firstRightMotor.configAllSettings(configuration);
        secondLeftMotor.configAllSettings(configuration);
        secondRightMotor.configAllSettings(configuration);
        firstLeftMotor.setNeutralMode(NeutralMode.Brake);
        firstRightMotor.setNeutralMode(NeutralMode.Brake);
        secondLeftMotor.setNeutralMode(NeutralMode.Brake);
        secondRightMotor.setNeutralMode(NeutralMode.Brake);

        firstRightMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        firstRightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        secondRightMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        secondRightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        firstLeftMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        firstLeftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        secondLeftMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        secondLeftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

        firstLeftMotor.enableVoltageCompensation(true);
        firstRightMotor.enableVoltageCompensation(true);
        secondLeftMotor.enableVoltageCompensation(true);
        secondRightMotor.enableVoltageCompensation(true);
        firstRightMotor.setInverted(true);
        secondRightMotor.setInverted(true);
    }

    @Override
    public void simulationPeriodic() {
        simulation.setInputVoltage(targetVoltage);
        simulation.update(0.020);
    }

    @Override
    public void periodic() {
        switch (mode) {
            case POSITION :
                if (isClimberZeroed()) {
                    firstLeftMotor.set(TalonFXControlMode.MotionMagic, targetHeight / SENSOR_POSITION_COEFFICIENT);
                    firstRightMotor.set(TalonFXControlMode.MotionMagic, targetHeight / SENSOR_POSITION_COEFFICIENT);
                    secondLeftMotor.set(TalonFXControlMode.MotionMagic, targetHeight / SENSOR_POSITION_COEFFICIENT);
                    secondRightMotor.set(TalonFXControlMode.MotionMagic, targetHeight / SENSOR_POSITION_COEFFICIENT);
                }
                break;
            case VOLTAGE :
                firstLeftMotor.set(TalonFXControlMode.PercentOutput, targetVoltage / 12.0);
                firstRightMotor.set(TalonFXControlMode.PercentOutput, targetVoltage / 12.0);
                secondLeftMotor.set(TalonFXControlMode.PercentOutput, targetVoltage / 12.0);
                secondRightMotor.set(TalonFXControlMode.PercentOutput, targetVoltage / 12.0);
                break;
        }
    }

    public double getTargetHeight() {
        if (mode == Mode.POSITION) {
            return targetHeight;
        }
        return 0.0;
    }

    public void setTargetHeight(double targetHeight) {
        this.targetHeight = MathUtils.clamp(targetHeight, Units.inchesToMeters(-10.0), MAX_HEIGHT);
        mode = Mode.POSITION;
    }

    public boolean isAtTargetPosition() {
        return Math.abs(getCurrentHeight() - getTargetHeight()) < ALLOWABLE_POSITION_ERROR;
    }

    public void setTargetVoltage(double targetVoltage) {
        this.targetVoltage = targetVoltage;
        mode = Mode.VOLTAGE;
    }

    public void setZeroed(boolean zeroed) {
        this.zeroed = zeroed;
    }

    public boolean isClimberZeroed() {
        return zeroed;
    }

    public double getCurrentHeight() {
        if (Robot.isSimulation()) {
            return simulation.getPositionMeters();
        } else {
            return firstLeftMotor.getSelectedSensorPosition() * SENSOR_POSITION_COEFFICIENT;
        }
    }

    public double getCurrentVelocity() {
        if (Robot.isSimulation()) {
            return simulation.getVelocityMetersPerSecond();
        } else {
            return firstLeftMotor.getSelectedSensorVelocity() * SENSOR_VELOCITY_COEFFICIENT;
        }
    }

    public void setZeroPosition() {
        if (Robot.isReal()) {
            firstLeftMotor.setSelectedSensorPosition(Units.inchesToMeters(-0.1875) / SENSOR_POSITION_COEFFICIENT);
            firstRightMotor.setSelectedSensorPosition(Units.inchesToMeters(-0.1875) / SENSOR_POSITION_COEFFICIENT);
            secondLeftMotor.setSelectedSensorPosition(Units.inchesToMeters(-0.1875) / SENSOR_POSITION_COEFFICIENT);
            secondRightMotor.setSelectedSensorPosition(Units.inchesToMeters(-0.1875) / SENSOR_POSITION_COEFFICIENT);
        }
    }

    public void setFastClimberConstraints(boolean fast) {
        if (fast) {
            firstLeftMotor
                    .configMotionCruiseVelocity(FAST_MOTION_CONSTRAINTS.maxVelocity / SENSOR_VELOCITY_COEFFICIENT);
            firstLeftMotor
                    .configMotionAcceleration(FAST_MOTION_CONSTRAINTS.maxAcceleration / SENSOR_VELOCITY_COEFFICIENT);
            firstRightMotor
                    .configMotionCruiseVelocity(FAST_MOTION_CONSTRAINTS.maxVelocity / SENSOR_VELOCITY_COEFFICIENT);
            firstRightMotor
                    .configMotionAcceleration(FAST_MOTION_CONSTRAINTS.maxAcceleration / SENSOR_VELOCITY_COEFFICIENT);
            secondLeftMotor
                    .configMotionCruiseVelocity(FAST_MOTION_CONSTRAINTS.maxVelocity / SENSOR_VELOCITY_COEFFICIENT);
            secondLeftMotor
                    .configMotionAcceleration(FAST_MOTION_CONSTRAINTS.maxAcceleration / SENSOR_VELOCITY_COEFFICIENT);
            secondRightMotor
                    .configMotionCruiseVelocity(FAST_MOTION_CONSTRAINTS.maxVelocity / SENSOR_VELOCITY_COEFFICIENT);
            secondRightMotor
                    .configMotionAcceleration(FAST_MOTION_CONSTRAINTS.maxAcceleration / SENSOR_VELOCITY_COEFFICIENT);
        } else {
            firstLeftMotor
                    .configMotionCruiseVelocity(SLOW_MOTION_CONSTRAINTS.maxVelocity / SENSOR_VELOCITY_COEFFICIENT);
            firstLeftMotor
                    .configMotionAcceleration(SLOW_MOTION_CONSTRAINTS.maxAcceleration / SENSOR_VELOCITY_COEFFICIENT);
            firstRightMotor
                    .configMotionCruiseVelocity(SLOW_MOTION_CONSTRAINTS.maxVelocity / SENSOR_VELOCITY_COEFFICIENT);
            firstRightMotor
                    .configMotionAcceleration(SLOW_MOTION_CONSTRAINTS.maxAcceleration / SENSOR_VELOCITY_COEFFICIENT);
            secondLeftMotor
                    .configMotionCruiseVelocity(SLOW_MOTION_CONSTRAINTS.maxVelocity / SENSOR_VELOCITY_COEFFICIENT);
            secondLeftMotor
                    .configMotionAcceleration(SLOW_MOTION_CONSTRAINTS.maxAcceleration / SENSOR_VELOCITY_COEFFICIENT);
            secondRightMotor
                    .configMotionCruiseVelocity(SLOW_MOTION_CONSTRAINTS.maxVelocity / SENSOR_VELOCITY_COEFFICIENT);
            secondRightMotor
                    .configMotionAcceleration(SLOW_MOTION_CONSTRAINTS.maxAcceleration / SENSOR_VELOCITY_COEFFICIENT);
        }
    }

    private enum Mode {
        POSITION, VOLTAGE
    }
}
