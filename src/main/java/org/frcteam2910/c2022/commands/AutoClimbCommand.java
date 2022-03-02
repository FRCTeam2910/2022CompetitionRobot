package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.frcteam2910.c2022.subsystems.ClimberSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class AutoClimbCommand extends SequentialCommandGroup {
    public AutoClimbCommand(ClimberSubsystem climber, ShooterSubsystem shooter) {
        // Climber is currently hooked on the mid rung, but the robot is on the ground

        // Prepare to transfer mid rung to hood
        addCommands(new ClimberToPointCommand(climber, ClimberSubsystem.HOOD_PASSAGE_HEIGHT)
                .alongWith(new SetHoodAngleCommand(shooter, ShooterSubsystem.HOOD_MAX_ANGLE)));

        // Transfer mid rung to hood
        addCommands(transferToHood(climber, shooter));
        // Move from mid rung to high rung
        addCommands(traverseToNextRung(climber, shooter));
        // Transfer high rung to hood
        addCommands(transferToHood(climber, shooter));
        // Move from high rung to traverse rung
        addCommands(traverseToNextRung(climber, shooter));
        // Transfer high rung to hood
        addCommands(transferToHood(climber, shooter));
    }

    private static Command transferToHood(ClimberSubsystem climber, ShooterSubsystem shooter) {
        SequentialCommandGroup group = new SequentialCommandGroup();

        // Move the climber to the transfer height
        group.addCommands(new ClimberToPointCommand(climber, ClimberSubsystem.HOOD_TRANSFER_HEIGHT));
        // Grab the rung with the hood
        group.addCommands(new SetHoodAngleCommand(shooter, ShooterSubsystem.HOOD_TRANSFER_ANGLE));
        // Release the rung with the climber so the hood can move freely
        group.addCommands(new ClimberToPointCommand(climber, ClimberSubsystem.HOOD_PASSAGE_HEIGHT));

        return group;
    }

    private static Command traverseToNextRung(ClimberSubsystem climber, ShooterSubsystem shooter) {
        SequentialCommandGroup group = new SequentialCommandGroup();

        // Angle the robot to extend the climber
        group.addCommands(new SetHoodAngleCommand(shooter, ShooterSubsystem.HOOD_TRAVERSE_EXTEND_ANGLE));
        // Extend the climber slightly past the next rung
        group.addCommands(new ClimberToPointCommand(climber, ClimberSubsystem.TRAVERSE_EXTEND_HEIGHT));
        // Angle the robot so the climber hooks will grab the next rung
        group.addCommands(new SetHoodAngleCommand(shooter, ShooterSubsystem.HOOD_TRAVERSE_RETRACT_ANGLE));
        // Retract the climber, and move the hood to the transfer position after the
        // climber grabs onto the next rung
        group.addCommands(new ClimberToPointCommand(climber, ClimberSubsystem.HOOD_PASSAGE_HEIGHT).alongWith(
                new WaitUntilCommand(() -> climber.getCurrentPosition() < ClimberSubsystem.TRAVERSE_RUNG_HEIGHT)
                        .andThen(new SetHoodAngleCommand(shooter, ShooterSubsystem.HOOD_MAX_ANGLE))));

        return group;
    }
}
