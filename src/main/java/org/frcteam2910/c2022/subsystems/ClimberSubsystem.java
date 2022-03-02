package org.frcteam2910.c2022.subsystems;

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
    public static final double MAX_HEIGHT = Units.feetToMeters(5.0);
    public static final double MID_RUNG_HEIGHT = Units.feetToMeters(4.5);
    public static final double TRAVERSE_EXTEND_HEIGHT = Units.feetToMeters(3.0);
    public static final double TRAVERSE_RUNG_HEIGHT = Units.feetToMeters(2.5);
    public static final double HOOD_PASSAGE_HEIGHT = Units.feetToMeters(1.0);
    public static final double HOOD_TRANSFER_HEIGHT = Units.feetToMeters(0.5);
    public static final double MIN_HEIGHT = Units.feetToMeters(0.0);

    private static final DCMotor MOTOR = DCMotor.getFalcon500(2);
    private static final double REDUCTION = 1.0 / 22.0;
    private static final double MASS = Units.lbsToKilograms(60.0);
    private static final double RADIUS = Units.inchesToMeters(1.0);

    private static final double VELOCITY_CONSTANT = 1.0 / (RADIUS * REDUCTION * MOTOR.KvRadPerSecPerVolt);
    private static final double ACCELERATION_CONSTANT = (MOTOR.rOhms * RADIUS * MASS * REDUCTION) / (MOTOR.KtNMPerAmp);

    private static final double ALLOWABLE_POSITION_ERROR = Units.inchesToMeters(2.0);

    private static final double SENSOR_POSITION_COEFFICIENT = REDUCTION * RADIUS / 2048.0;
    private static final double SENSOR_VELOCITY_COEFFICIENT = SENSOR_POSITION_COEFFICIENT / 10.0;

    private static final MotionProfile.Constraints MOTION_CONSTRAINTS = new MotionProfile.Constraints(
            11.0 * VELOCITY_CONSTANT * 0.2, 11.0 * ACCELERATION_CONSTANT * 0.8);

    private final LinearSystem<N2, N1, N1> plant = LinearSystemId.identifyPositionSystem(VELOCITY_CONSTANT,
            ACCELERATION_CONSTANT);
    private final ElevatorSim simulation = new ElevatorSim(plant, MOTOR, 1.0 / REDUCTION, RADIUS, 0.0,
            MAX_HEIGHT * 1.1);
    private final TalonFX leftMotor = new TalonFX(Constants.CLIMBER_LEFT_MOTOR_PORT);
    private final TalonFX rightMotor = new TalonFX(Constants.CLIMBER_RIGHT_MOTOR_PORT);

    private Mode mode = Mode.VOLTAGE;
    private double targetHeight = 0.0;
    private double targetVoltage = 0.0;

    public ClimberSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Climber");
        shuffleboardTab.addNumber("Target Height", () -> Units.metersToFeet(getTargetHeight()));
        shuffleboardTab.addNumber("Height", () -> Units.metersToFeet(getCurrentHeight()));
        shuffleboardTab.addNumber("Velocity", () -> Units.metersToFeet(getCurrentVelocity()));

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.motionCruiseVelocity = MOTION_CONSTRAINTS.maxVelocity / SENSOR_VELOCITY_COEFFICIENT;
        configuration.motionAcceleration = MOTION_CONSTRAINTS.maxAcceleration / SENSOR_VELOCITY_COEFFICIENT;
        configuration.slot0.kP = 0.0;
        configuration.slot0.kI = 0.0;
        configuration.slot0.kD = 0.0;
        configuration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        configuration.supplyCurrLimit.currentLimit = 40.0;
        configuration.supplyCurrLimit.enable = true;
        configuration.voltageCompSaturation = 12.0;

        leftMotor.configAllSettings(configuration);
        rightMotor.configAllSettings(configuration);

        rightMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        rightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        leftMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        leftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

        leftMotor.enableVoltageCompensation(true);
        rightMotor.enableVoltageCompensation(true);
        rightMotor.setInverted(true);
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
                leftMotor.set(TalonFXControlMode.MotionMagic, targetHeight / SENSOR_POSITION_COEFFICIENT);
                rightMotor.set(TalonFXControlMode.MotionMagic, targetHeight / SENSOR_POSITION_COEFFICIENT);
                break;
            case VOLTAGE :
                leftMotor.set(TalonFXControlMode.PercentOutput, targetVoltage / 12.0);
                rightMotor.set(TalonFXControlMode.PercentOutput, targetVoltage / 12.0);
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
        this.targetHeight = MathUtils.clamp(targetHeight, MIN_HEIGHT, MAX_HEIGHT);
        mode = Mode.POSITION;
    }

    public boolean isAtTargetPosition() {
        return Math.abs(getCurrentHeight() - getTargetHeight()) < ALLOWABLE_POSITION_ERROR;
    }

    public void setTargetVoltage(double targetVoltage) {
        this.targetVoltage = targetVoltage;
        mode = Mode.VOLTAGE;
    }

    public double getCurrentHeight() {
        if (Robot.isSimulation()) {
            return simulation.getPositionMeters();
        } else {
            return leftMotor.getSelectedSensorPosition() * SENSOR_POSITION_COEFFICIENT;
        }
    }

    public double getCurrentVelocity() {
        if (Robot.isSimulation()) {
            return simulation.getVelocityMetersPerSecond();
        } else {
            return leftMotor.getSelectedSensorVelocity() * SENSOR_VELOCITY_COEFFICIENT;
        }
    }

    public void setZeroPosition() {
        if (Robot.isReal()) {
            leftMotor.setSelectedSensorPosition(0.0);
            rightMotor.setSelectedSensorPosition(0.0);
        }
    }

    private enum Mode {
        POSITION, VOLTAGE
    }
}
