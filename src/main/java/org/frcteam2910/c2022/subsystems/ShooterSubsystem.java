package org.frcteam2910.c2022.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2022.Constants;
import org.frcteam2910.c2022.Robot;
import org.frcteam2910.common.control.MotionProfileFollower;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.motion.MotionProfile;
import org.frcteam2910.common.motion.TrapezoidalMotionProfile;

public class ShooterSubsystem implements Subsystem {
    private static final double HOOD_MOMENT_OF_INERTIA = Units.lbsToKilograms(Units.inchesToMeters(450));
    private static final double HOOD_GEAR_REDUCTION = (14.0 / 54.0) * (18.0 / 38.0) * (20.0 / 36.0) * (10.0 / 220.0);
    private static final double FLYWHEEL_GEAR_REDUCTION = 1.0;
    private static final double FLYWHEEL_ALLOWABLE_ERROR = Units.rotationsPerMinuteToRadiansPerSecond(100);

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
    private final PIDController flywheelVelocityController = new PIDController(0.1, 0.0, 0.0);

    private double flywheelVoltage;
    private double hoodVoltage;
    private boolean isHoodZeroed = false;
    private double targetFlywheelSpeed;

    private final MotionProfileFollower hoodMotionFollower = new MotionProfileFollower(
            new PidController(new PidConstants(100.0, 0.0, 1.0)), HOOD_VELOCITY_CONSTANT, HOOD_ACCELERATION_CONSTANT);

    public ShooterSubsystem() {
        // hoodAngleMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Shooter");
        shuffleboardTab.addNumber("Flywheel Speed",
                () -> Units.radiansPerSecondToRotationsPerMinute(getFlywheelVelocity()));
        shuffleboardTab.addNumber("Hood Target Angle", () -> Math.toDegrees(getHoodTargetPosition()));
        shuffleboardTab.addNumber("Hood Angle", () -> Math.toDegrees(getHoodAngle()));
        shuffleboardTab.addNumber("Hood Velocity", () -> Math.toDegrees(getHoodVelocity()));
        shuffleboardTab.addNumber("Last Motion Profile State Velocity",
                () -> Math.toDegrees(hoodMotionFollower.getLastState().map(state -> state.velocity).orElse(0.0)));
        shuffleboardTab.addNumber("Hood Voltage", () -> hoodVoltage);

        flywheelPrimaryMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        flywheelPrimaryMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        flywheelPrimaryMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        flywheelSecondaryMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        flywheelSecondaryMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        hoodAngleMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        hoodAngleMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

        flywheelPrimaryMotor.configVoltageCompSaturation(12.0);
        flywheelSecondaryMotor.configVoltageCompSaturation(12.0);
        flywheelPrimaryMotor.enableVoltageCompensation(true);
        flywheelSecondaryMotor.enableVoltageCompensation(true);

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

    public double getHoodVoltage() {
        return hoodVoltage;
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
        if (Robot.isSimulation()) {
            return Math.abs(getHoodTargetPosition() - hoodSim.getAngleRads()) < Constants.HOOD_ALLOWABLE_ERROR;
        } else {
            return Math.abs(getHoodTargetPosition() - getHoodAngle()) < Constants.HOOD_ALLOWABLE_ERROR;
        }
    }

    public void setTargetFlywheelSpeed(double targetFlywheelSpeed) {
        this.targetFlywheelSpeed = targetFlywheelSpeed;
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
        double startingPosition = hoodMotionFollower.getLastState().map(state -> state.position).orElse(getHoodAngle());
        double startingVelocity = hoodMotionFollower.getLastState().map(state -> state.velocity)
                .orElse(getHoodVelocity());

        hoodMotionFollower.follow(new TrapezoidalMotionProfile(new MotionProfile.Goal(getHoodAngle(), 0.0),
                new MotionProfile.Goal(position, 0.0), MOTION_CONSTRAINTS));
    }

    public double getHoodTargetPosition() {
        MotionProfile profile = hoodMotionFollower.getCurrentMotionProfile();
        if (profile != null) {
            return profile.getEnd().position;
        } else {
            return 0.0;
        }
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

    @Override
    public void periodic() {
        final double now = Timer.getFPGATimestamp();
        final double dt = Robot.kDefaultPeriod;

        if (isHoodZeroed) {
            hoodVoltage = hoodMotionFollower.update(getHoodAngle(), now, dt);
        }

        hoodAngleMotor.set(TalonFXControlMode.PercentOutput, hoodVoltage / 12);

        flywheelVoltage = flywheelVelocityController.calculate(getFlywheelVelocity(), targetFlywheelSpeed)
                + targetFlywheelSpeed * FLYWHEEL_VELOCITY_CONSTANT;

        flywheelPrimaryMotor.set(TalonFXControlMode.PercentOutput, flywheelVoltage / 12.0);
        flywheelSecondaryMotor.set(TalonFXControlMode.PercentOutput, flywheelVoltage / 12.0);
    }
}