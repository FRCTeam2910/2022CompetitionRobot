package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.frcteam2910.c2022.subsystems.ClimberSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;
import org.frcteam2910.c2022.util.ClimbChooser;

public class AutoClimbCommand extends SequentialCommandGroup {
    private final ClimberSubsystem climber;
    private final ShooterSubsystem shooter;
    private final ClimbTypeSupplier climbTypeSupplier;
    private boolean hasAddedCommands;

    @FunctionalInterface
    public interface ClimbTypeSupplier {
        ClimbChooser.ClimbType getClimbType();
    }

    public AutoClimbCommand(ClimberSubsystem climber, ShooterSubsystem shooter, ClimbTypeSupplier climbTypeSupplier) {
        this.climber = climber;
        this.shooter = shooter;
        this.climbTypeSupplier = climbTypeSupplier;
    }

    @Override
    public void initialize() {
        if (!hasAddedCommands) {
            hasAddedCommands = true;
            ClimbChooser.ClimbType climbType = climbTypeSupplier.getClimbType();
            // Climber is currently hooked on the mid rung, but the robot is on the ground
            addCommands(new InstantCommand(shooter::disableFlywheel));

            // Prepare to transfer mid rung to hood
            addCommands(new ClimberToPointCommand(climber, ClimberSubsystem.HOOD_TRANSFER_HEIGHT, false, true)
                    .alongWith(new SetHoodAngleCommand(shooter, ShooterSubsystem.HOOD_PREPARE_TRANSFER_ANGLE)));

            // Transfer mid rung to hood
            addCommands(transferToHood(climber, shooter));
            if (climbType != ClimbChooser.ClimbType.MID) {
                // Move from mid rung to high rung
                if (climbType == ClimbChooser.ClimbType.HIGH_PARTWAY) {
                    addCommands(traverseToNextRung(climber, shooter, false, true));
                    addCommands(new SetHoodAngleCommand(shooter, ShooterSubsystem.HOOD_MIN_ANGLE));
                } else {
                    addCommands(traverseToNextRung(climber, shooter, false));
                }
                if (climbType != ClimbChooser.ClimbType.HIGH_PARTWAY) {
                    // Transfer high rung to hood
                    addCommands(transferToHood(climber, shooter));
                    if (climbType != ClimbChooser.ClimbType.HIGH_HOOK) {
                        // Move from high rung to traverse rung
                        if (climbType == ClimbChooser.ClimbType.TRAVERSAL_PARTWAY) {
                            addCommands(traverseToNextRung(climber, shooter, true, true));
                        } else {
                            addCommands(traverseToNextRung(climber, shooter, true, false));
                        }
                        if (climbType != ClimbChooser.ClimbType.TRAVERSAL_PARTWAY) {
                            addCommands(new ClimberToPointCommand(climber, ClimberSubsystem.HOOD_PASSAGE_HEIGHT, false,
                                    false));
                            // Transfer traverse rung to hood
                            addCommands(transferToHood(climber, shooter));
                        }
                    }
                }
            }
            addCommands(new InstantCommand(() -> climber.setTargetVoltage(0.0)));
            addCommands(new WaitCommand(5).perpetually());
            super.initialize();
        }
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

    private static Command traverseToNextRung(ClimberSubsystem climber, ShooterSubsystem shooter, boolean transversal) {
        return traverseToNextRung(climber, shooter, transversal, false);
    }

    private static Command traverseToNextRung(ClimberSubsystem climber, ShooterSubsystem shooter, boolean transversal,
            boolean partway) {
        SequentialCommandGroup group = new SequentialCommandGroup();

        if (!transversal) {
            group.addCommands(
                    new SetHoodAngleCommand(shooter, ShooterSubsystem.HOOD_TRAVERSE_EXTEND_ANGLE_HIGH, true, true)
                            .alongWith(new ClimberToPointCommand(climber, ClimberSubsystem.TRAVERSE_EXTEND_HEIGHT)));
        } else {
            // Angle the robot to extend the climber
            group.addCommands(new SetHoodAngleCommand(shooter, ShooterSubsystem.HOOD_TRAVERSE_EXTEND_ANGLE_TRANVERSE,
                    false, true));
            // Extend the climber slightly past the next rung
            group.addCommands(new ClimberToPointCommand(climber, ClimberSubsystem.TRAVERSE_EXTEND_HEIGHT));
        }
        // Angle the robot so the climber hooks will grab the next rung
        group.addCommands(new SetHoodAngleCommand(shooter, ShooterSubsystem.HOOD_TRAVERSE_RETRACT_ANGLE, false, true));

        if (transversal) {
            if (partway) {
                group.addCommands(
                        new ClimberToPointCommand(climber, ClimberSubsystem.TRAVERSE_RUNG_PARTWAY_HEIGHT, false, false)
                                .alongWith(new SetHoodAngleCommand(shooter,
                                        ShooterSubsystem.HOOD_PREPARE_TRANSFER_ANGLE, true, true)));
                group.addCommands(new SetHoodAngleCommand(shooter, ShooterSubsystem.HOOD_MIN_ANGLE));
            } else {
                group.addCommands(new ClimberToPointCommand(climber, ClimberSubsystem.HOOD_PASSAGE_HEIGHT, false, false)
                        .alongWith(new SetHoodAngleCommand(shooter, ShooterSubsystem.HOOD_PREPARE_TRANSFER_ANGLE, true,
                                true)));
            }
        } else {
            // Retract the climber, and move the hood to the transfer position after the
            // climber grabs onto the next rung
            if (!partway) {
                group.addCommands(new ClimberToPointCommand(climber, ClimberSubsystem.HOOD_PASSAGE_HEIGHT, false, false)
                        .alongWith(new SetHoodAngleCommand(shooter, ShooterSubsystem.HOOD_PREPARE_TRANSFER_ANGLE, true,
                                true)));
            } else {
                group.addCommands(
                        new ClimberToPointCommand(climber, ClimberSubsystem.TRAVERSE_RUNG_PARTWAY_HEIGHT, false, false)
                                .alongWith(new SetHoodAngleCommand(shooter,
                                        ShooterSubsystem.HOOD_PREPARE_TRANSFER_ANGLE, true, true)));
            }
        }

        return group;
    }
}
