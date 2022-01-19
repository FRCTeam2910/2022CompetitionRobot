package org.frcteam2910.c2022;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Robot extends TimedRobot {
    private final DCMotor gearbox = DCMotor.getVex775Pro(4);
    private final ElevatorSim elevator = new ElevatorSim(gearbox, 10.0, 4.0, Units.inchesToMeters(2), Units.inchesToMeters(1), Units.inchesToMeters(8));
    private final PWMTalonSRX motor = new PWMTalonSRX(0);

    private final Mechanism2d mech2d = new Mechanism2d(100, 120);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 10);
    private final MechanismLigament2d position = mech2dRoot.append(new MechanismLigament2d("Position", 0, 90));
    private final MechanismLigament2d motorOutput = mech2dRoot.append(new MechanismLigament2d("Motor Output", 0, 45, 10, new Color8Bit(150, 0, 255)));
    private final Joystick joystick = new Joystick(0);

    @Override
    public void robotInit() {
        SmartDashboard.putData("Elevator Sim", mech2d);
    }

    public void robotPeriodic(){
        if(Robot.isSimulation()){
            elevator.setInput(motor.get());
            motor.set(-joystick.getY());
            elevator.update(0.020);
            position.setLength(Units.metersToInches(elevator.getPositionMeters()) * 10);
            motorOutput.setLength(motor.get() * 100);
            System.out.println(position.getLength());
        }
    }
}
