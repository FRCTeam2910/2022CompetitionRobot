package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.common.robot.UpdateManager;

public class FeederSubsystem implements Subsystem, UpdateManager.Updatable {

    private TalonFX motor = new TalonFX(Constants.FEEDER_MOTOR_PORT);
    private AnalogInput fullSensor = new AnalogInput(Constants.FEEDER_IS_FULL_SENSOR_PORT);
    private AnalogInput intakeBallSensor = new AnalogInput(Constants.FEEDER_INTAKE_BALL_SENSOR_PORT);

    private boolean moveFirstBallFast = true;

    private final NetworkTableEntry fullEntry;
    private final NetworkTableEntry hasBallEntry;
    private final NetworkTableEntry motorSpeedEntry;

    public FeederSubsystem() {
        motor.setInverted(false);
        motor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        motor.setNeutralMode(NeutralMode.Brake);

        ShuffleboardTab tab = Shuffleboard.getTab("Feeder");
        fullEntry = tab.add("Is feeder full", false).withPosition(0, 0).withSize(1, 1).getEntry();
        hasBallEntry = tab.add("Does shooter have ball", false).withPosition(0, 1).withSize(1, 1).getEntry();
        motorSpeedEntry = tab.add("Motor Speed", 0.0).withPosition(1, 0).withSize(1, 1).getEntry();
    }

    public void spinMotor(double speed) {
        motor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public boolean isFull() {
        return fullSensor.getVoltage() < 0.1;
    }

    public boolean shouldAdvance() {
        if (isFull()) {
            return false;
        }
        return isBallAtIntake();
    }

    @Override
    public void update(double time, double dt) {

    }

    @Override
    public void periodic() {
        fullEntry.setBoolean(isFull());
        hasBallEntry.setBoolean(isBallAtIntake());
        motorSpeedEntry.setDouble(motor.getSensorCollection().getIntegratedSensorVelocity());
    }

    public boolean isBallAtIntake() {
        return intakeBallSensor.getVoltage() < 0.1;
    }

    public boolean isFifthBallAtIntake() {
        return isFull() && isBallAtIntake();
    }

    public void setMoveFirstBallFast(boolean target) {
        this.moveFirstBallFast = target;
    }

    public boolean getMoveFirstBallFast() {
        return moveFirstBallFast;
    }
}
