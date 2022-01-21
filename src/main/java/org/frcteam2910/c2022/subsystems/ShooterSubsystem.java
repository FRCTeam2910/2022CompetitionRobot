package org.frcteam2910.c2022.subsystems;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class ShooterSubsystem implements Subsystem {
    private static final double HOOD_MOMENT_OF_INTERTIA = Units.lbsToKilograms(Units.inchesToMeters(450));
    private static final double HOOD_GEAR_REDUCTION = 85.0;
    private final FlywheelSim flywheel = new FlywheelSim(DCMotor.getFalcon500(2), 1.0, Units.inchesToMeters(6));
    private final LinearSystem hoodPlant = LinearSystemId.createSingleJointedArmSystem(DCMotor.getFalcon500(2), HOOD_MOMENT_OF_INTERTIA, HOOD_GEAR_REDUCTION);
    private final LinearSystemSim hoodSim = new LinearSystemSim(hoodPlant);
    private double voltage;
    private double hoodVoltage;

    public ShooterSubsystem(){
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Shooter");
        shuffleboardTab.addNumber("Flywheel Speed", () -> Units.radiansPerSecondToRotationsPerMinute(getFlywheelSpeed()));
        shuffleboardTab.addNumber("Hood Angle", () -> Math.toDegrees(getHoodAngle()));
    }

    public double getFlywheelSpeed(){
        return flywheel.getAngularVelocityRadPerSec();
    }

    public double getHoodAngle(){
        return hoodSim.getOutput(0);
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void setHoodVoltage(double hoodVoltage){
        this.hoodVoltage = hoodVoltage;
    }



    public void simulationPeriodic() {

            flywheel.setInputVoltage(voltage);
            flywheel.update(0.02);

            hoodSim.setInput(hoodVoltage);
            hoodSim.update(0.02);

        }
    }

