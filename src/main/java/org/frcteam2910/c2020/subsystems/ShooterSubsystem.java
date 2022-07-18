package org.frcteam2910.c2020.subsystems;

import java.util.OptionalDouble;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.robot.UpdateManager;

public class ShooterSubsystem implements Subsystem, UpdateManager.Updatable {
    private static final double HOOD_SENSOR_COEFFICIENT = ((2.0 * Math.PI) * Constants.SHOOTER_HOOD_GEAR_RATIO
            / 4096.0);

    private static final double FLYWHEEL_POSITION_SENSOR_COEFFICIENT = 1.0 / 2048.0;
    private static final double FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT = FLYWHEEL_POSITION_SENSOR_COEFFICIENT
            * (1000.0 / 100.0) * (60.0 / 1.0);

    private static final double FLYWHEEL_P = 0.25;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.0;

    /**
     * Flywheel regression constants
     * <p>
     * Found by doing an exponential regression using the following points: (0.25,
     * 1480) (0.3, 1780) (0.5, 3000) (0.6, 3600) (0.7, 4235) (0.8, 4835)
     */
    private static final double FLYWHEEL_FF_CONSTANT = 0.00189;
    private static final double FLYWHEEL_STATIC_FRICTION_CONSTANT = 0.469;

    private static final double FLYWHEEL_CURRENT_LIMIT = 5.0;
    private static final int HOOD_CURRENT_LIMIT = 15;

    private static final double FLYWHEEL_ALLOWABLE_ERROR = 200.0;

    private final TalonFX flywheelMotor1 = new TalonFX(Constants.SHOOTER_DRIVE_MOTOR_PORT_1);
    private final TalonFX flywheelMotor2 = new TalonFX(Constants.SHOOTER_DRIVE_MOTOR_PORT_2);

    private final TalonSRX angleMotor = new TalonSRX(Constants.SHOOTER_ANGLE_MOTOR_PORT);

    private final PidController hoodController = new PidController(new PidConstants(4.0, 0.0, 0.0));

    private final NetworkTableEntry hoodAngleEntry;
    private final NetworkTableEntry flyWheelMotor1SpeedEntry;
    private final NetworkTableEntry flywheelMotor1VoltageEntry;
    private final NetworkTableEntry flywheelMotor1CurrentEntry;
    private final NetworkTableEntry flyWheelMotor2SpeedEntry;
    private final NetworkTableEntry flywheelMotor2VoltageEntry;
    private final NetworkTableEntry flywheelMotor2CurrentEntry;

    private HoodControlMode hoodControlMode = HoodControlMode.DISABLED;
    private OptionalDouble hoodTargetPosition = OptionalDouble.empty();

    public ShooterSubsystem() {
        flywheelMotor1.configFactoryDefault();
        flywheelMotor2.configFactoryDefault();

        TalonFXConfiguration flyWheelConfiguration = new TalonFXConfiguration();
        flyWheelConfiguration.slot0.kP = FLYWHEEL_P;
        flyWheelConfiguration.slot0.kI = FLYWHEEL_I;
        flyWheelConfiguration.slot0.kD = FLYWHEEL_D;
        flyWheelConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor
                .toFeedbackDevice();
        flyWheelConfiguration.supplyCurrLimit.currentLimit = FLYWHEEL_CURRENT_LIMIT;
        flyWheelConfiguration.supplyCurrLimit.enable = true;
        flyWheelConfiguration.voltageCompSaturation = 11.5;

        flywheelMotor1.configAllSettings(flyWheelConfiguration);
        flywheelMotor2.configAllSettings(flyWheelConfiguration);

        flywheelMotor1.enableVoltageCompensation(false);
        flywheelMotor2.enableVoltageCompensation(false);

        TalonSRXConfiguration hoodConfiguration = new TalonSRXConfiguration();
        hoodConfiguration.feedbackNotContinuous = false;
        hoodConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Absolute;
        hoodConfiguration.continuousCurrentLimit = HOOD_CURRENT_LIMIT;

        angleMotor.configAllSettings(hoodConfiguration);
        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setInverted(true);
        angleMotor.setSensorPhase(false);

        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        hoodAngleEntry = tab.add("hood angle", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
        tab.addNumber("Hood Target Angle", () -> Math.toDegrees(getHoodTargetAngle().orElse(Double.NaN)))
                .withPosition(0, 1).withSize(1, 1);
        tab.addNumber("Hood Raw Encoder", angleMotor::getSelectedSensorPosition).withPosition(0, 2).withSize(1, 1);
        flyWheelMotor1SpeedEntry = tab.add("Wheel 1 Speed", 0.0).withPosition(1, 0).withSize(1, 1).getEntry();
        flywheelMotor1VoltageEntry = tab.add("Wheel 1 Voltage", 0.0).withPosition(2, 0).withSize(1, 1).getEntry();
        flywheelMotor1CurrentEntry = tab.add("Wheel 1 Current", 0.0).withPosition(3, 0).withSize(1, 1).getEntry();
        flyWheelMotor2SpeedEntry = tab.add("Wheel 2 Speed", 0.0).withPosition(1, 1).withSize(1, 1).getEntry();
        flywheelMotor2VoltageEntry = tab.add("Wheel 2 Voltage", 0.0).withPosition(2, 1).withSize(1, 1).getEntry();
        flywheelMotor2CurrentEntry = tab.add("Wheel 2 Current", 0.0).withPosition(3, 1).withSize(1, 1).getEntry();
        tab.addBoolean("Is Flywheel at Target", this::isFlywheelAtTargetVelocity).withPosition(4, 1).withSize(1, 1);
        tab.addNumber("Flywheel Target", this::getFlywheelTargetVelocity).withPosition(4, 0).withSize(1, 1);
    }

    public void setFlywheelCurrentLimitEnabled(boolean enabled) {
        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration();
        config.currentLimit = FLYWHEEL_CURRENT_LIMIT;
        config.enable = enabled;
        flywheelMotor1.configSupplyCurrentLimit(config, 0);
        flywheelMotor2.configSupplyCurrentLimit(config, 0);
    }

    public double getHoodAngle() {
        return (angleMotor.getSelectedSensorPosition() % 4096.0) * HOOD_SENSOR_COEFFICIENT
                + Constants.SHOOTER_HOOD_OFFSET;
    }

    public OptionalDouble getHoodTargetAngle() {
        return hoodTargetPosition;
    }

    public void setHoodTargetAngle(double angle) {
        hoodControlMode = HoodControlMode.POSITION;
        hoodTargetPosition = OptionalDouble.of(angle);
    }

    public void shootFlywheel(double speed) {
        double feedforward = (FLYWHEEL_FF_CONSTANT * speed + FLYWHEEL_STATIC_FRICTION_CONSTANT)
                / RobotController.getBatteryVoltage();

        flywheelMotor1.set(ControlMode.Velocity, -speed / FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT,
                DemandType.ArbitraryFeedForward, -feedforward);
        flywheelMotor2.set(ControlMode.Velocity, speed / FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT,
                DemandType.ArbitraryFeedForward, feedforward);
    }

    public void setFlywheelOutput(double percentage) {
        flywheelMotor1.set(ControlMode.PercentOutput, -percentage);
        flywheelMotor2.set(ControlMode.PercentOutput, percentage);
    }

    public void stopFlywheel() {
        flywheelMotor1.set(ControlMode.Disabled, 0);
        flywheelMotor2.set(ControlMode.Disabled, 0);
    }

    @Override
    public void update(double time, double dt) {

    }

    @Override
    public void periodic() {
        switch (hoodControlMode) {
            case DISABLED :
                angleMotor.set(TalonSRXControlMode.Disabled, 0.0);
                break;
            case POSITION :
                if (getHoodTargetAngle().isEmpty()) {
                    angleMotor.set(TalonSRXControlMode.Disabled, 0.0);
                } else {
                    double targetAngle = getHoodTargetAngle().getAsDouble();
                    targetAngle = MathUtils.clamp(targetAngle, Constants.SHOOTER_HOOD_MIN_ANGLE,
                            Constants.SHOOTER_HOOD_MAX_ANGLE);

                    hoodController.setSetpoint(targetAngle);
                    angleMotor.set(TalonSRXControlMode.PercentOutput, hoodController.calculate(getHoodAngle(), 0.02));
                }
                break;
        }

        hoodAngleEntry.setDouble(Math.toDegrees(getHoodAngle()));
        flyWheelMotor1SpeedEntry.setDouble(-flywheelMotor1.getSensorCollection().getIntegratedSensorVelocity()
                * FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT);
        flywheelMotor1VoltageEntry.setDouble(-flywheelMotor1.getMotorOutputVoltage());
        flywheelMotor1CurrentEntry.setDouble(flywheelMotor1.getSupplyCurrent());
        flyWheelMotor2SpeedEntry.setDouble(flywheelMotor2.getSensorCollection().getIntegratedSensorVelocity()
                * FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT);
        flywheelMotor2VoltageEntry.setDouble(flywheelMotor2.getMotorOutputVoltage());
        flywheelMotor2CurrentEntry.setDouble(flywheelMotor2.getSupplyCurrent());
    }

    public double getFlywheelPosition() {
        return -flywheelMotor1.getSensorCollection().getIntegratedSensorPosition()
                * FLYWHEEL_POSITION_SENSOR_COEFFICIENT;
    }

    public double getFlywheelVelocity() {
        return -flywheelMotor1.getSensorCollection().getIntegratedSensorVelocity()
                * FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT;
    }

    public double getFlywheelTargetVelocity() {
        return -flywheelMotor1.getClosedLoopTarget() * FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT;
    }

    public void resetFlywheelPosition() {
        flywheelMotor1.getSensorCollection().setIntegratedSensorPosition(0.0, 0);
    }

    public boolean isFlywheelAtTargetVelocity() {
        return MathUtils.epsilonEquals(getFlywheelVelocity(), getFlywheelTargetVelocity(), FLYWHEEL_ALLOWABLE_ERROR);
    }

    public void disableHood() {
        hoodControlMode = HoodControlMode.DISABLED;
        hoodTargetPosition = OptionalDouble.empty();
    }

    public enum HoodControlMode {
        DISABLED, POSITION
    }
}
