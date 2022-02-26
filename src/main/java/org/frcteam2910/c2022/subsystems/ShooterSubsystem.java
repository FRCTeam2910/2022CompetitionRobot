package org.frcteam2910.c2022.subsystems;

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
    private static final double HOOD_GEAR_REDUCTION = 1.0 / 85.0;
    private static final double FLYWHEEL_ALLOWABLE_ERROR = 0.1;
    private static final double SENSOR_UNITS_PER_RADIAN = 1; // TODO: Update to correctly convert sensor ticks to
                                                                // radians

    private static final DCMotor HOOD_MOTOR = DCMotor.getFalcon500(1);
    private static final double VELOCITY_CONSTANT = 1.0 / (HOOD_GEAR_REDUCTION * HOOD_MOTOR.KvRadPerSecPerVolt);
    private static final double ACCELERATION_CONSTANT = (HOOD_MOTOR.rOhms * HOOD_MOMENT_OF_INERTIA
            * HOOD_GEAR_REDUCTION) / HOOD_MOTOR.KtNMPerAmp;
    private static final double SENSOR_POSITION_COEFFICIENT = HOOD_GEAR_REDUCTION / 2048.0;
    private static final double SENSOR_VELOCITY_COEFFICIENT = SENSOR_POSITION_COEFFICIENT / 10.0;

    private static final MotionProfile.Constraints MOTION_CONSTRAINTS = new MotionProfile.Constraints(
            11.0 * VELOCITY_CONSTANT * 0.2, 11.0 * ACCELERATION_CONSTANT * 0.8);

    private final TalonFX hoodAngleMotor = new TalonFX(Constants.HOOD_MOTOR_PORT);
    private final LinearSystem<N2, N1, N1> hoodPlant = LinearSystemId
            .createSingleJointedArmSystem(DCMotor.getFalcon500(2), HOOD_MOMENT_OF_INERTIA, HOOD_GEAR_REDUCTION);
    private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(hoodPlant, HOOD_MOTOR,
            1.0 / HOOD_GEAR_REDUCTION, 0.0, 0.0, Math.PI, 0.0, false);

    private final FlywheelSim flywheel = new FlywheelSim(DCMotor.getFalcon500(2), 1.0, Units.inchesToMeters(6));
    private final TalonFX flywheelPrimaryMotor = new TalonFX(Constants.FLYWHEEL_PRIMARY_MOTOR_PORT);
    private final TalonFX flywheelSecondaryMotor = new TalonFX(Constants.FLYWHEEL_SECONDARY_MOTOR_PORT);
    private final PIDController flywheelVelocityController = new PIDController(0.5, 0.0, 0.0);

    private double flywheelVoltage;
    private double hoodVoltage;
    private boolean isHoodZeroed;
    private double targetFlywheelSpeed;

    private final MotionProfileFollower motionFollower = new MotionProfileFollower(
            new PidController(new PidConstants(1.0, 0.0, 0.0)), VELOCITY_CONSTANT, ACCELERATION_CONSTANT);

    public ShooterSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Shooter");
        shuffleboardTab.addNumber("Flywheel Speed",
                () -> Units.radiansPerSecondToRotationsPerMinute(getFlywheelSpeed()));
        shuffleboardTab.addNumber("Hood Angle", () -> Math.toDegrees(getHoodAngle()));

        flywheelPrimaryMotor.configVoltageCompSaturation(12.0);
        flywheelSecondaryMotor.configVoltageCompSaturation(12.0);
        flywheelPrimaryMotor.enableVoltageCompensation(true);
        flywheelSecondaryMotor.enableVoltageCompensation(true);
    }

    public double getFlywheelSpeed() {
        return flywheel.getAngularVelocityRadPerSec();
    }

    public double getHoodAngle() {
        if (Robot.isSimulation()) {
            return hoodSim.getAngleRads();
        } else {
            return hoodAngleMotor.getSelectedSensorPosition() * SENSOR_POSITION_COEFFICIENT;
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
            return hoodAngleMotor.getSelectedSensorVelocity() * SENSOR_VELOCITY_COEFFICIENT;
        }
    }

    public void setHoodZeroed(boolean zeroed) {
        this.isHoodZeroed = zeroed;
    }

    public boolean isHoodAtTargetAngle() {
        if (Robot.isSimulation()) {
            return Math.abs(getHoodTargetPosition() - hoodSim.getAngleRads()) < Constants.HOOD_ALLOWABLE_ERROR;
        } else {
            return Math.abs(getHoodTargetPosition() - (hoodAngleMotor.getSelectedSensorPosition()
                    / SENSOR_UNITS_PER_RADIAN)) < Constants.HOOD_ALLOWABLE_ERROR;
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
            return flywheelPrimaryMotor.getActiveTrajectoryVelocity();
        }
    }

    public boolean isFlywheelAtTargetSpeed() {
        return Math.abs(getFlywheelVelocity() - targetFlywheelSpeed) < FLYWHEEL_ALLOWABLE_ERROR;
    }

    public void setHoodTargetPosition(double position) {
        motionFollower.follow(new TrapezoidalMotionProfile(new MotionProfile.Goal(getHoodAngle(), getHoodVelocity()),
                new MotionProfile.Goal(position, 0.0), MOTION_CONSTRAINTS));
    }

    public double getHoodTargetPosition() {
        MotionProfile profile = motionFollower.getCurrentMotionProfile();
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
        hoodAngleMotor.setSelectedSensorPosition(position / SENSOR_POSITION_COEFFICIENT);
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

        hoodVoltage = motionFollower.update(getHoodAngle(), now, dt);

        flywheelVoltage = flywheelVelocityController.calculate(getFlywheelVelocity(), targetFlywheelSpeed);

        // flywheelPrimaryMotor.set(TalonFXControlMode.PercentOutput, flywheelVoltage /
        // 12.0);
        // flywheelSecondaryMotor.set(TalonFXControlMode.PercentOutput, flywheelVoltage
        // / 12.0);
    }
}