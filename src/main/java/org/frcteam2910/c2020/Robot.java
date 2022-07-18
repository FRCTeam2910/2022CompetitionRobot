package org.frcteam2910.c2020;

import java.io.IOException;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frcteam2910.c2020.commands.HomeClimberCommand;
import org.frcteam2910.c2020.commands.HomeHoodMotorCommand;
import org.frcteam2910.common.Logger;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.robot.drivers.Limelight;

public class Robot extends TimedRobot {
    private static Robot instance = null;

    private static final Logger LOGGER = new Logger(Robot.class);

    private static final byte[] COMPETITION_BOT_MAC_ADDRESS = new byte[]{0x00, (byte) 0x80, 0x2f, 0x28, (byte) 0xc3,
            0x33};
    private static final byte[] PRACTICE_BOT_MAC_ADDRESS = new byte[]{0x00, (byte) 0x80, 0x2f, 0x22, (byte) 0xd7,
            (byte) 0xba};

    private static boolean competitionBot;
    private static boolean practiceBot;

    private RobotContainer robotContainer = new RobotContainer();
    private UpdateManager updateManager = new UpdateManager(robotContainer.getDrivetrainSubsystem(),
            robotContainer.getFeederSubsystem(), robotContainer.getIntakeSubsystem(),
            robotContainer.getShooterSubsystem());

    static {
        List<byte[]> macAddresses;
        try {
            macAddresses = getMacAddresses();
        } catch (IOException e) {
            // Don't crash, just log the stacktrace and continue without any mac addresses.
            LOGGER.error(e);
            macAddresses = List.of();
        }

        for (byte[] macAddress : macAddresses) {
            // First check if we are the competition bot
            if (Arrays.compare(COMPETITION_BOT_MAC_ADDRESS, macAddress) == 0) {
                competitionBot = true;
                break;
            }

            // Next check if we are the practice bot
            if (Arrays.compare(PRACTICE_BOT_MAC_ADDRESS, macAddress) == 0) {
                practiceBot = true;
                break;
            }
        }

        if (!competitionBot && !practiceBot) {
            String[] macAddressStrings = macAddresses.stream().map(Robot::macToString).toArray(String[]::new);

            SmartDashboard.putStringArray("MAC Addresses", macAddressStrings);
            SmartDashboard.putString("Competition Bot MAC Address", macToString(COMPETITION_BOT_MAC_ADDRESS));
            SmartDashboard.putString("Practice Bot MAC Address", macToString(PRACTICE_BOT_MAC_ADDRESS));

            // If something goes terribly wrong we still want to use the competition bot
            // stuff in competition.
            competitionBot = true;
        }

        SmartDashboard.putBoolean("Competition Bot", competitionBot);
    }

    public Robot() {
        instance = this;
    }

    public static Robot getInstance() {
        return instance;
    }

    public static boolean isCompetitionBot() {
        return competitionBot;
    }

    public static boolean isPracticeBot() {
        return practiceBot;
    }

    /**
     * Gets the MAC addresses of all present network adapters.
     *
     * @return the MAC addresses of all network adapters.
     */
    private static List<byte[]> getMacAddresses() throws IOException {
        List<byte[]> macAddresses = new ArrayList<>();

        Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();

        NetworkInterface networkInterface;
        while (networkInterfaces.hasMoreElements()) {
            networkInterface = networkInterfaces.nextElement();

            byte[] address = networkInterface.getHardwareAddress();
            if (address == null) {
                continue;
            }

            macAddresses.add(address);
        }

        return macAddresses;
    }

    private static String macToString(byte[] address) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < address.length; i++) {
            if (i != 0) {
                builder.append(':');
            }
            builder.append(String.format("%02X", address[i]));
        }
        return builder.toString();
    }

    @Override
    public void robotInit() {
        updateManager.startLoop(5.0e-3);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        double distFromARedAngle = robotContainer.getDrivetrainSubsystem().getPose().rotation
                .rotateBy(Rotation2.fromDegrees(180).inverse()).toRadians();
        double distFromABlueAngle = robotContainer.getDrivetrainSubsystem().getPose().rotation
                .rotateBy(Rotation2.fromDegrees(90).inverse()).toRadians();
        double distFromBRedAngle = robotContainer.getDrivetrainSubsystem().getPose().rotation
                .rotateBy(Rotation2.fromDegrees(45).inverse()).toRadians();
        double distFromBBlueAngle = robotContainer.getDrivetrainSubsystem().getPose().rotation
                .rotateBy(Rotation2.fromDegrees(-45).inverse()).toRadians();

        if (distFromARedAngle > Math.PI) {
            distFromARedAngle = (2 * Math.PI) - distFromARedAngle;
        }
        if (distFromABlueAngle > Math.PI) {
            distFromABlueAngle = (2 * Math.PI) - distFromABlueAngle;
        }
        if (distFromBRedAngle > Math.PI) {
            distFromBRedAngle = (2 * Math.PI) - distFromBRedAngle;
        }
        if (distFromBBlueAngle > Math.PI) {
            distFromBBlueAngle = (2 * Math.PI) - distFromBBlueAngle;
        }

        double smallestDist = Math.min(distFromARedAngle,
                Math.min(distFromABlueAngle, Math.min(distFromBBlueAngle, distFromBRedAngle)));

        if (distFromARedAngle == smallestDist) {
            SmartDashboard.putString("Autonomous settings", "Path A Red");
        } else if (distFromABlueAngle == smallestDist) {
            SmartDashboard.putString("Autonomous settings", "Path A Blue");
        } else if (distFromBRedAngle == smallestDist) {
            SmartDashboard.putString("Autonomous settings", "Path B Red");
        } else if (distFromBBlueAngle == smallestDist) {
            SmartDashboard.putString("Autonomous settings", "Path B Blue");
        }
    }

    @Override
    public void autonomousInit() {
        new HomeHoodMotorCommand(robotContainer.getShooterSubsystem()).schedule();

        robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.DEFAULT);
        robotContainer.getDrivetrainSubsystem().resetPose(RigidTransform2.ZERO);
        // robotContainer.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO);

        robotContainer.getAutonomousCommand().schedule();
    }

    @Override
    public void testInit() {
        // new TestModeShooterCommand(robotContainer.getShooterSubsystem()).schedule();
    }

    @Override
    public void testPeriodic() {
        // robotContainer.getShooterSubsystem().disableHood();
    }

    @Override
    public void disabledPeriodic() {
        // robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.OFF);
    }

    @Override
    public void teleopInit() {
        if (!robotContainer.getShooterSubsystem().isHoodHomed()) {
            new HomeHoodMotorCommand(robotContainer.getShooterSubsystem()).schedule();
        }

        new HomeClimberCommand(robotContainer.getClimberSubsystem(), robotContainer.getSuperstructure()).schedule();
    }
}
