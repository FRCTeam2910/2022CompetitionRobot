package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frcteam2910.c2022.subsystems.ClimberSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class TransferringBarsCommandGroup extends SequentialCommandGroup {
    public static final double CLIMBER_TOP = 1.0;
    public static final double HOOD_ANGLE_FOR_CLIMBER_EXTENSION_TO_NEXT_BAR = Math.toRadians(45);

    public final ClimberSubsystem climber;
    public final ShooterSubsystem shooter;

    public TransferringBarsCommandGroup(ClimberSubsystem climber, ShooterSubsystem shooter) {
        this.climber = climber;
        this.shooter = shooter;
        addCommands(
            //Move climber up off bar while also moving the hood to orient the robot correctly
            new ClimberToPointCommand(climber, CLIMBER_TOP)
                .alongWith(new WaitCommand(0.25))
                    .andThen(new SetHoodAngleCommand(shooter, HOOD_ANGLE_FOR_CLIMBER_EXTENSION_TO_NEXT_BAR))
        );
    }
}
