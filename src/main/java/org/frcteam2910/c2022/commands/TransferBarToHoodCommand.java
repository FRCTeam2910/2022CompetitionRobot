package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2022.subsystems.ClimberSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class TransferBarToHoodCommand extends SequentialCommandGroup {
    public static final double CLIMBER_BOTTOM = 0.25;
    public static final double HOOD_HOOK_ANGLE = Math.toRadians(90);
    public static final double HOOD_ANGLE_READY_TO_HOOK = Math.toRadians(125);

    public final ClimberSubsystem climber;
    public final ShooterSubsystem shooter;

    public TransferBarToHoodCommand(ClimberSubsystem climber, ShooterSubsystem shooter) {
        this.climber = climber;
        this.shooter = shooter;
        addCommands(new SetHoodAngleCommand(shooter, HOOD_ANGLE_READY_TO_HOOK),
                // Move Climber down so that hood can hook
                new ClimberToPointCommand(climber, CLIMBER_BOTTOM),
                // Hook hood
                new SetHoodAngleCommand(shooter, HOOD_HOOK_ANGLE));
    }
}
