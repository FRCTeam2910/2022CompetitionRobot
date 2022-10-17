package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.util.DetectedColor;
import org.frcteam2910.common.robot.UpdateManager;

public class WheelOfFortuneSubsystem implements Subsystem, UpdateManager.Updatable {

    private final double RED_MIN = Constants.WHEEL_OF_FORTUNE_RED_HUE - (Constants.WHEEL_OF_FORTUNE_RED_HUE - 0) / 2;
    private final double RED_MAX = Constants.WHEEL_OF_FORTUNE_RED_HUE
            + (Constants.WHEEL_OF_FORTUNE_YELLOW_HUE - Constants.WHEEL_OF_FORTUNE_RED_HUE) / 2;
    private final double GREEN_MIN = Constants.WHEEL_OF_FORTUNE_GREEN_HUE
            - (Constants.WHEEL_OF_FORTUNE_GREEN_HUE - Constants.WHEEL_OF_FORTUNE_YELLOW_HUE) / 2;
    private final double GREEN_MAX = Constants.WHEEL_OF_FORTUNE_GREEN_HUE
            + (Constants.WHEEL_OF_FORTUNE_BLUE_HUE - Constants.WHEEL_OF_FORTUNE_GREEN_HUE) / 2;
    private final double BLUE_MIN = Constants.WHEEL_OF_FORTUNE_BLUE_HUE
            - (Constants.WHEEL_OF_FORTUNE_BLUE_HUE - Constants.WHEEL_OF_FORTUNE_GREEN_HUE) / 2;
    private final double BLUE_MAX = Constants.WHEEL_OF_FORTUNE_BLUE_HUE
            + (210 - Constants.WHEEL_OF_FORTUNE_BLUE_HUE) / 2;

    private static final double SPINNER_POSITION_COEFFICIENT = 0.0;
    private static final double SPINNER_INTEGRAL_COEFFICIENT = 0.0;
    private static final double SPINNER_DERIVATIVE_COEFFICIENT = 0.0;
    // private static final double SENSOR_COEFFICIENT = 9.0 * (32.0 / 2.0);
    private static final double SENSOR_COEFFICIENT = 1.0;

    private static final int PROXIMITY_CUTOFF_VALUE = 1024;

    public static final double SPINNER_REVOLUTIONS_PER_WHEEL_SECTION = 1.0;
    //
    // private final I2C.Port i2cPort = I2C.Port.kOnboard;
    // private final ColorSensorV3 COLOR_SENSOR = new ColorSensorV3(i2cPort);

    // private CANSparkMax motor = new
    // CANSparkMax(Constants.WHEEL_OF_FORTUNE_MOTOR_PORT, MotorType.kBrushless);
    //
    // private CANEncoder encoder = motor.getEncoder();
    // private CANPIDController pidController = motor.getPIDController();
    private TalonSRX motor = new TalonSRX(Constants.WHEEL_OF_FORTUNE_MOTOR_PORT);

    private Solenoid deploySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
            Constants.WHEEL_OF_FORTUNE_DEPLOY_SOLENOID_PORT);

    private DetectedColor detectedColor;
    private final NetworkTableEntry colorEntry;

    private int proximityValue;

    public WheelOfFortuneSubsystem() {
        // encoder.setPositionConversionFactor(SENSOR_COEFFICIENT);
        //
        // pidController.setP(SPINNER_POSITION_COEFFICIENT);
        // pidController.setI(SPINNER_INTEGRAL_COEFFICIENT);
        // pidController.setD(SPINNER_DERIVATIVE_COEFFICIENT);
        TalonSRXConfiguration configuration = new TalonSRXConfiguration();
        configuration.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
        configuration.primaryPID.selectedFeedbackCoefficient = SENSOR_COEFFICIENT;
        configuration.slot0.kP = SPINNER_POSITION_COEFFICIENT;
        configuration.slot0.kI = SPINNER_INTEGRAL_COEFFICIENT;
        configuration.slot0.kD = SPINNER_DERIVATIVE_COEFFICIENT;

        motor.configAllSettings(configuration);

        ShuffleboardTab tab = Shuffleboard.getTab("Wheel of Fortune");
        colorEntry = tab.add("Color", DetectedColor.GREEN.toString()).withPosition(0, 0).withSize(1, 1).getEntry();
        tab.addNumber("encoder position", () -> getEncoderPosition()).withPosition(1, 0).withSize(1, 1);
    }

    @Override
    public void periodic() {
        // Color colorFromSensor = COLOR_SENSOR.getColor();
        Color colorFromSensor = Color.kAqua;
        double hueColor = calculateHue(colorFromSensor);
        detectedColor = calculateDetectedColor(hueColor);
        //
        // proximityValue = COLOR_SENSOR.getProximity();
        proximityValue = 0;

        colorEntry.setString(detectedColor.toString());
    }

    public void spin(double numRevolutions) {
        // pidController.setReference(numRevolutions, ControlType.kPosition);
        motor.set(ControlMode.MotionMagic, numRevolutions);
    }

    public boolean isCloseToColorSensor() {
        return proximityValue >= PROXIMITY_CUTOFF_VALUE;
    }

    public void setMotorSpeed(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    public void stopMotor() {
        motor.set(ControlMode.PercentOutput, 0.0);
    }

    public void resetEncoderPosition() {
        motor.setSelectedSensorPosition(0);
    }

    public double getEncoderPosition() {
        return motor.getSelectedSensorPosition();
    }

    private double calculateHue(Color colorFromSensor) {

        // RGB to Hue formula
        double cMax = Math.max(Math.max(colorFromSensor.red, colorFromSensor.green), colorFromSensor.blue);
        double cMin = Math.min(Math.min(colorFromSensor.red, colorFromSensor.green), colorFromSensor.blue);
        double delta = cMax - cMin;

        if (colorFromSensor.red > colorFromSensor.green && colorFromSensor.red > colorFromSensor.blue) {// Red is bigger
            return 60 * (((colorFromSensor.green - colorFromSensor.blue) / delta) % 6);
        } else if (colorFromSensor.green > colorFromSensor.red && colorFromSensor.green > colorFromSensor.blue) {// Green
                                                                                                                    // is
                                                                                                                    // bigger
            return 60 * (((colorFromSensor.blue - colorFromSensor.red) / delta) + 2);
        } else {// Blue is bigger
            return 60 * (((colorFromSensor.red - colorFromSensor.green) / delta) + 4);
        }
    }

    private DetectedColor calculateDetectedColor(double colorHue) {
        if (colorHue > RED_MIN && colorHue < RED_MAX) {
            return DetectedColor.RED;
        } else if (colorHue > GREEN_MIN && colorHue < GREEN_MAX) {
            return DetectedColor.GREEN;
        } else if (colorHue > BLUE_MIN && colorHue < BLUE_MAX) {
            return DetectedColor.BLUE;
        } else {
            return DetectedColor.YELLOW;
        }
    }

    public DetectedColor getDetectedColor() {
        return detectedColor;
    }

    public void extendSolenoid() {
        deploySolenoid.set(true);
    }

    public void retractSolenoid() {
        deploySolenoid.set(false);
    }

    @Override
    public void update(double time, double dt) {

    }

}
