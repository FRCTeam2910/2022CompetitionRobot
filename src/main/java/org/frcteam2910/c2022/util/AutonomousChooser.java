package org.frcteam2910.c2022.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2022.RobotContainer;


public class AutonomousChooser {
    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(){
        autonomousModeChooser.setDefaultOption("1 Ball Auto", AutonomousMode.ONE_BALL);
        autonomousModeChooser.addOption("3 Ball Auto", AutonomousMode.THREE_BALL);
        autonomousModeChooser.addOption("5 Ball Auto", AutonomousMode.FIVE_BALL);
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser(){
        return autonomousModeChooser;
    }
    public Command get1BallAuto(){
        SequentialCommandGroup command = new SequentialCommandGroup();

        //Add code here

        return command;
    }

    public Command get3BallAuto(){
        SequentialCommandGroup command = new SequentialCommandGroup();

        //Add code here

        return command;
    }

    public Command get5BallAuto(){
        SequentialCommandGroup command = new SequentialCommandGroup();

        //Add code here

        return command;
    }


    public void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Pose2d pose){
        command.addCommands(new InstantCommand(() -> container.getDrivetrain().setPose(
                pose)));
    }

    private enum AutonomousMode {
        ONE_BALL,
        THREE_BALL,
        FIVE_BALL
    }
}
