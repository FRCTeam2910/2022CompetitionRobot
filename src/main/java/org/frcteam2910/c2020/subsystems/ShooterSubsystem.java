package org.frcteam2910.c2020.subsystems;

import java.util.OptionalDouble;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.robot.UpdateManager;

public class ShooterSubsystem implements Subsystem, UpdateManager.Updatable {
    private static final double FLYWHEEL_POSITION_SENSOR_COEFFICIENT = 1.0 / 2048.0
            * Constants.BOTTOM_FLYWHEEL_GEAR_RATIO;
    private static final double FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT = FLYWHEEL_POSITION_SENSOR_COEFFICIENT
            * (1000.0 / 100.0) * (60.0);// in terms of talonfx counts
    private static final double FLYWHEEL_FEEDFORWARD_COEFFICIENT = 0.0012148;
    private static final double FLYWHEEL_STATIC_FRICTION_CONSTANT = 0.5445;

    private static final double FLYWHEEL_ALLOWABLE_ERROR = 300.0;

    private static final double FLYWHEEL_P = 0.5;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.0;

    private static final double FLYWHEEL_CURRENT_LIMIT = 10.0;

    private static final double HOOD_ANGLE_P = 0.5;
    private static final double HOOD_ANGLE_I = 0;
    private static final double HOOD_ANGLE_D = 5;
    private static final double HOOD_CURRENT_LIMIT = 15.0;

    private final TalonFX flywheelPrimaryMotor = new TalonFX(Constants.FLYWHEEL_PRIMARY_MOTOR_PORT);
    private final TalonFX flywheelSecondaryMotor = new TalonFX(Constants.FLYWHEEL_SECONDARY_MOTOR_PORT);
    private final TalonFX flywheelTertiaryMotor = new TalonFX(Constants.FLYWHEEL_TERTIARY_MOTOR_PORT);

    private final TalonFX hoodAngleMotor = new TalonFX(Constants.HOOD_MOTOR_PORT);

    private final NetworkTableEntry hoodAngleEntry;

    private boolean isHoodHomed;

    private HoodControlMode hoodControlMode = HoodControlMode.DISABLED;
    private double hoodTargetPosition = Double.NaN;
    private double hoodPercentOutput = 0.0;

    public ShooterSubsystem() {
        isHoodHomed = false;

        flywheelPrimaryMotor.configFactoryDefault();
        flywheelSecondaryMotor.configFactoryDefault();

        // Save CAN bandwidth
        flywheelSecondaryMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        flywheelSecondaryMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

        hoodAngleMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        hoodAngleMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

        TalonFXConfiguration flywheelConfiguration = new TalonFXConfiguration();
        flywheelConfiguration.slot0.kP = FLYWHEEL_P;
        flywheelConfiguration.slot0.kI = FLYWHEEL_I;
        flywheelConfiguration.slot0.kD = FLYWHEEL_D;
        flywheelConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor
                .toFeedbackDevice();
        flywheelConfiguration.supplyCurrLimit.currentLimit = FLYWHEEL_CURRENT_LIMIT;
        flywheelConfiguration.supplyCurrLimit.enable = true;
        flywheelConfiguration.voltageCompSaturation = 11.5;

        flywheelPrimaryMotor.configAllSettings(flywheelConfiguration);
        flywheelSecondaryMotor.configAllSettings(flywheelConfiguration);
        flywheelTertiaryMotor.configAllSettings(flywheelConfiguration);

        flywheelPrimaryMotor.enableVoltageCompensation(false);
        flywheelSecondaryMotor.enableVoltageCompensation(false);
        flywheelTertiaryMotor.enableVoltageCompensation(false);

        flywheelSecondaryMotor.follow(flywheelPrimaryMotor);
        flywheelTertiaryMotor.follow(flywheelPrimaryMotor);
        flywheelTertiaryMotor.setInverted(TalonFXInvertType.Clockwise);

        TalonFXConfiguration hoodConfiguration = new TalonFXConfiguration();
        hoodConfiguration.slot0.kP = HOOD_ANGLE_P;
        hoodConfiguration.slot0.kI = HOOD_ANGLE_I;
        hoodConfiguration.slot0.kD = HOOD_ANGLE_D;
        hoodConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        hoodConfiguration.supplyCurrLimit.currentLimit = HOOD_CURRENT_LIMIT;
        hoodConfiguration.supplyCurrLimit.enable = true;

        hoodAngleMotor.configAllSettings(hoodConfiguration);
        hoodAngleMotor.setNeutralMode(NeutralMode.Coast);
        hoodAngleMotor.setSensorPhase(false);
        hoodAngleMotor.setInverted(true);

        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        hoodAngleEntry = tab.add("hood angle", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
        tab.addNumber("Hood Target Angle", () -> Math.toDegrees(getHoodTargetAngle().orElse(Double.NaN)))
                .withPosition(0, 1).withSize(1, 1);
        tab.addNumber("Hood Raw Encoder", hoodAngleMotor::getSelectedSensorPosition).withPosition(0, 2).withSize(1, 1);
        tab.addBoolean("Is Bottom Flywheel at Target", this::isFlywheelAtTargetVelocity).withPosition(2, 1).withSize(2,
                1);
        tab.addNumber("Bottom Flywheel Target", this::getFlywheelTargetVelocity).withPosition(2, 0).withSize(2, 1);
        tab.addNumber("Bottom Flywheel Speed", this::getFlywheelVelocity).withPosition(2, 2).withSize(2, 1);
        tab.addNumber("Angle Error", this::angleTargetError).withPosition(1, 1).withSize(1, 1);
        tab.addBoolean("Hood Homed", this::isHoodHomed).withPosition(1, 0).withSize(1, 1);

    }

    public boolean isHoodAtTargetAngle() {
        OptionalDouble targetAngle = getHoodTargetAngle();
        double currentAngle = getHoodMotorAngle();

        if (targetAngle.isEmpty()) {
            return false;
        }

        return MathUtils.epsilonEquals(targetAngle.getAsDouble(), currentAngle, Math.toRadians(1.0));
    }

    public void setFlywheelCurrentLimitEnabled(boolean enabled) {
        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration();
        config.currentLimit = FLYWHEEL_CURRENT_LIMIT;
        config.enable = enabled;
        flywheelPrimaryMotor.configSupplyCurrentLimit(config, 0);
        flywheelSecondaryMotor.configSupplyCurrentLimit(config, 0);
        flywheelTertiaryMotor.configSupplyCurrentLimit(config, 0);
    }

    public OptionalDouble getHoodTargetAngle() {
        if (Double.isFinite(hoodTargetPosition)) {
            return OptionalDouble.of(hoodTargetPosition);
        } else {
            return OptionalDouble.empty();
        }
    }

    public void setHoodTargetAngle(double angle) {
        hoodControlMode = HoodControlMode.POSITION;
        hoodTargetPosition = angle;
    }

    public void shootFlywheel(double speed) {
        double feedforward = (FLYWHEEL_FEEDFORWARD_COEFFICIENT * speed + FLYWHEEL_STATIC_FRICTION_CONSTANT)
                / RobotController.getBatteryVoltage();

        flywheelPrimaryMotor.set(ControlMode.Velocity, -speed / FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT,
                DemandType.ArbitraryFeedForward, -feedforward);
    }

    public void setFlywheelOutput(double percentage) {
        flywheelPrimaryMotor.set(ControlMode.PercentOutput, -percentage);
    }

    public void stopFlywheel() {
        flywheelPrimaryMotor.set(ControlMode.Disabled, 0);
    }

    @Override
    public void update(double time, double dt) {
    }

    @Override
    public void periodic() {
        switch (hoodControlMode) {
            case DISABLED :
                hoodAngleMotor.set(TalonFXControlMode.Disabled, 0.0);
                break;
            case POSITION :
                if (!isHoodHomed) {
                    break;
                }

                if (getHoodTargetAngle().isEmpty()) {
                    hoodAngleMotor.set(TalonFXControlMode.Disabled, 0.0);
                } else {
                    double targetAngle = getHoodTargetAngle().getAsDouble();
                    targetAngle = MathUtils.clamp(targetAngle, Constants.HOOD_MIN_ANGLE, Constants.HOOD_MAX_ANGLE);

                    hoodAngleMotor.set(TalonFXControlMode.Position, angleToTalonUnits(targetAngle));

                }
                break;
            case PERCENT_OUTPUT :
                this.hoodAngleMotor.set(ControlMode.PercentOutput, hoodPercentOutput);
                break;
        }

        hoodAngleEntry.setDouble(Math.toDegrees(getHoodMotorAngle()));
    }

    public double getFlywheelPosition() {
        return -flywheelPrimaryMotor.getSensorCollection().getIntegratedSensorPosition()
                * FLYWHEEL_POSITION_SENSOR_COEFFICIENT;
    }

    public double getFlywheelVelocity() {
        return -flywheelPrimaryMotor.getSensorCollection().getIntegratedSensorVelocity()
                * FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT;
    }

    public double getFlywheelTargetVelocity() {
        return -flywheelPrimaryMotor.getClosedLoopTarget() * FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT;
    }

    public void resetFlywheelPosition() {
        flywheelPrimaryMotor.getSensorCollection().setIntegratedSensorPosition(0.0, 0);
    }

    public boolean isFlywheelAtTargetVelocity() {
        return MathUtils.epsilonEquals(getFlywheelVelocity(), getFlywheelTargetVelocity(), FLYWHEEL_ALLOWABLE_ERROR);
    }

    private double angleTargetError() {
        if (getHoodTargetAngle().isPresent()) {
            return Math.toDegrees(getHoodTargetAngle().getAsDouble() - getHoodMotorAngle());
        }

        return 0.0;

    }

    public void disableHood() {
        hoodControlMode = HoodControlMode.DISABLED;
        hoodTargetPosition = Double.NaN;
    }

    public double getHoodVelocity() {
        return -hoodAngleMotor.getSensorCollection().getIntegratedSensorVelocity() * 10 / 2048 * (2 * Math.PI)
                * Constants.HOOD_MOTOR_TO_HOOD_GEAR_RATIO;
    }

    public void zeroHoodMotor() {
        this.isHoodHomed = true;

        double sensorPosition = (Constants.HOOD_MAX_ANGLE + Math.toRadians(1)) * 2048 / (2 * Math.PI)
                * Constants.HOOD_MOTOR_TO_HOOD_GEAR_RATIO;
        hoodAngleMotor.setSelectedSensorPosition((int) sensorPosition);
    }

    private double talonUnitsToHoodAngle(double talonUnits) {
        return -talonUnits / 2048 * (2 * Math.PI) / Constants.HOOD_MOTOR_TO_HOOD_GEAR_RATIO;
    }

    private double angleToTalonUnits(double angle) {
        return angle * 2048 / (2 * Math.PI) * Constants.HOOD_MOTOR_TO_HOOD_GEAR_RATIO;
    }

    public void setHoodMotorPower(double percent) {
        hoodControlMode = HoodControlMode.PERCENT_OUTPUT;
        hoodPercentOutput = percent;
    }

    public boolean isHoodHomed() {
        return isHoodHomed;
    }

    public void setHoodHomed(boolean target) {
        this.isHoodHomed = target;
    }

    public double getHoodMotorAngle() {
        return talonUnitsToHoodAngle(hoodAngleMotor.getSensorCollection().getIntegratedSensorPosition());
    }

    public enum HoodControlMode {
        DISABLED, POSITION, PERCENT_OUTPUT
    }
}
