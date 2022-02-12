package org.frcteam2910.c2022.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
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

public class ClimberSubsystem implements Subsystem {
    private static final DCMotor MOTOR = DCMotor.getFalcon500(2);
    private static final double REDUCTION = 1.0 / 22.0;
    private static final double MASS = Units.lbsToKilograms(60.0);
    private static final double RADIUS = Units.inchesToMeters(1.0);

    private static final double MAX_HEIGHT = 1.0;

    private static final double ALLOWABLE_POSITION_ERROR = Units.inchesToMeters(2.0);

    private static final double POSITION_COEFFICIENT = 1;
    private static final double VELOCITY_COEFFICIENT = POSITION_COEFFICIENT / 10.0;

    private final LinearSystem<N2, N1, N1> plant = LinearSystemId.createElevatorSystem(MOTOR, MASS, RADIUS,
            1.0 / REDUCTION);
    private final ElevatorSim simulation = new ElevatorSim(plant, MOTOR, 1.0 / REDUCTION, RADIUS, 0.0,
            MAX_HEIGHT * 1.1);
    private final TalonFX motor = new TalonFX(Constants.CLIMBER_MOTOR_PORT);
    private final PIDController positionPID = new PIDController(25.0, 0.0, 0.0);

    private final Mechanism2d mech2d = new Mechanism2d(100, 120);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 10);
    private final MechanismLigament2d position = mech2dRoot.append(new MechanismLigament2d("Position", 0, 90));
    private final MechanismLigament2d motorOutput = mech2dRoot
            .append(new MechanismLigament2d("Motor Output", 0, 45, 10, new Color8Bit(150, 0, 255)));

    private Mode mode = Mode.VOLTAGE;
    private double targetPosition = 0.0;
    private double targetVoltage = 0.0;
    private double inputVoltage = 0.0;

    public ClimberSubsystem() {
        SmartDashboard.putData("Elevator Sim", mech2d);
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Climber");
        shuffleboardTab.addNumber("velocity", simulation::getVelocityMetersPerSecond);
        shuffleboardTab.addNumber("height", simulation::getPositionMeters);
        shuffleboardTab.addNumber("target height", () -> targetPosition);
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
        switch (mode) {
            case POSITION :
                inputVoltage = positionPID.calculate(getCurrentPosition(), targetPosition);
                break;
            case VOLTAGE :
                inputVoltage = targetVoltage;
                break;
        }

        motor.set(TalonFXControlMode.PercentOutput, inputVoltage / 12);
        position.setLength(simulation.getPositionMeters() * 100);
        motorOutput.setLength((inputVoltage / 12.0 + 1) * 50);
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        if (mode != Mode.POSITION) {
            positionPID.reset();
            mode = Mode.POSITION;
        }
    }

    public boolean isAtTargetPosition() {
        return Math.abs(getCurrentPosition() - targetPosition) < ALLOWABLE_POSITION_ERROR;
    }

    public void setTargetVoltage(double targetVoltage) {
        this.targetVoltage = targetVoltage;
        mode = Mode.VOLTAGE;
    }

    public double getCurrentPosition() {
        if (Robot.isSimulation()) {
            return simulation.getPositionMeters();
        } else {
            return motor.getSelectedSensorPosition() * POSITION_COEFFICIENT;
        }
    }

    public double getCurrentVelocity() {
        if (Robot.isSimulation()) {
            return simulation.getVelocityMetersPerSecond();
        } else {
            return motor.getSelectedSensorVelocity() * VELOCITY_COEFFICIENT;
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
