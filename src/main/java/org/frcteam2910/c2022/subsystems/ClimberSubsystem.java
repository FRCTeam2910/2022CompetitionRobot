package org.frcteam2910.c2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
import org.frcteam2910.c2022.Robot;
import org.frcteam2910.common.control.MotionProfileFollower;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.motion.MotionProfile;
import org.frcteam2910.common.motion.TrapezoidalMotionProfile;

public class ClimberSubsystem implements Subsystem {
    private static final DCMotor MOTOR = DCMotor.getFalcon500(2);
    private static final double REDUCTION = 1.0 / 22.0;
    private static final double MASS = Units.lbsToKilograms(60.0);
    private static final double RADIUS = Units.inchesToMeters(1.0);

    private static final double VELOCITY_CONSTANT = 1.0 / (RADIUS * REDUCTION * MOTOR.KvRadPerSecPerVolt);
    private static final double ACCELERATION_CONSTANT = (MOTOR.rOhms * RADIUS * MASS * REDUCTION) / (MOTOR.KtNMPerAmp);

    private static final double MAX_HEIGHT = 1.0;

    private static final double ALLOWABLE_POSITION_ERROR = Units.inchesToMeters(2.0);

    private static final double SENSOR_POSITION_COEFFICIENT = REDUCTION * RADIUS / 2048.0;
    private static final double SENSOR_VELOCITY_COEFFICIENT = SENSOR_POSITION_COEFFICIENT / 10.0;

    private static final MotionProfile.Constraints MOTION_CONSTRAINTS = new MotionProfile.Constraints(
            11.0 * VELOCITY_CONSTANT * 0.2, 11.0 * ACCELERATION_CONSTANT * 0.8);

    private final LinearSystem<N2, N1, N1> plant = LinearSystemId.identifyPositionSystem(VELOCITY_CONSTANT,
            ACCELERATION_CONSTANT);
    private final ElevatorSim simulation = new ElevatorSim(plant, MOTOR, 1.0 / REDUCTION, RADIUS, 0.0,
            MAX_HEIGHT * 1.1);
    private final TalonFX motor = new TalonFX(Constants.CLIMBER_LEFT_MOTOR_PORT);

    private final Mechanism2d mech2d = new Mechanism2d(100, 120);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 10);
    private final MechanismLigament2d position = mech2dRoot.append(new MechanismLigament2d("Position", 0, 90));
    private final MechanismLigament2d motorOutput = mech2dRoot
            .append(new MechanismLigament2d("Motor Output", 0, 45, 10, new Color8Bit(150, 0, 255)));

    private final MotionProfileFollower motionFollower = new MotionProfileFollower(
            new PidController(new PidConstants(25.0, 0.0, 0.0)), VELOCITY_CONSTANT, ACCELERATION_CONSTANT);

    private Mode mode = Mode.VOLTAGE;
    private double targetVoltage = 0.0;
    private double inputVoltage = 0.0;

    public ClimberSubsystem() {
        SmartDashboard.putData("Elevator Sim", mech2d);
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Climber");
        shuffleboardTab.addNumber("velocity", simulation::getVelocityMetersPerSecond);
        shuffleboardTab.addNumber("height", simulation::getPositionMeters);
        shuffleboardTab.addNumber("target height", this::getTargetPosition);
        shuffleboardTab.addNumber("voltage", () -> inputVoltage);

        motor.configVoltageCompSaturation(12.0);
        motor.enableVoltageCompensation(true);
    }

    @Override
    public void simulationPeriodic() {
        simulation.setInputVoltage(inputVoltage);
        simulation.update(0.020);
    }

    @Override
    public void periodic() {
        final double now = Timer.getFPGATimestamp();
        final double dt = Robot.kDefaultPeriod;

        switch (mode) {
            case POSITION :
                inputVoltage = motionFollower.update(getCurrentPosition(), now, dt);

                break;
            case VOLTAGE :
                inputVoltage = targetVoltage;
                break;
        }

        // motor.set(TalonFXControlMode.PercentOutput, inputVoltage / 12);
        position.setLength(simulation.getPositionMeters() * 100);
        motorOutput.setLength((inputVoltage / 12.0 + 1) * 50);
    }

    public double getTargetPosition() {
        if (mode == Mode.POSITION) {
            return motionFollower.getCurrentMotionProfile().getEnd().position;
        }
        return 0.0;
    }

    public void setTargetPosition(double targetPosition) {
        motionFollower
                .follow(new TrapezoidalMotionProfile(new MotionProfile.Goal(getCurrentPosition(), getCurrentVelocity()),
                        new MotionProfile.Goal(targetPosition, 0.0), MOTION_CONSTRAINTS));
        mode = Mode.POSITION;
    }

    public boolean isAtTargetPosition() {
        return Math.abs(getCurrentPosition() - getTargetPosition()) < ALLOWABLE_POSITION_ERROR;
    }

    public void setTargetVoltage(double targetVoltage) {
        this.targetVoltage = targetVoltage;
        mode = Mode.VOLTAGE;
    }

    public double getCurrentPosition() {
        if (Robot.isSimulation()) {
            return simulation.getPositionMeters();
        } else {
            return motor.getSelectedSensorPosition() * SENSOR_POSITION_COEFFICIENT;
        }
    }

    public double getCurrentVelocity() {
        if (Robot.isSimulation()) {
            return simulation.getVelocityMetersPerSecond();
        } else {
            return motor.getSelectedSensorVelocity() * SENSOR_VELOCITY_COEFFICIENT;
        }
    }

    public void setZeroPosition() {
        if (Robot.isReal()) {
            motor.setSelectedSensorPosition(0.0);
        }
    }

    private enum Mode {
        VOLTAGE, POSITION
    }
}
