package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2022.subsystems.ClimberSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class GetHoodReadyToHookCommandGroup extends SequentialCommandGroup {
    public static final double CLIMBER_HEIGHT_TO_ALLOW_HOOD_PASSAGE = 0.5;
    public static final double HOOD_ANGLE_READY_TO_HOOK = Math.toRadians(125);

    public final ClimberSubsystem climber;
    public final ShooterSubsystem shooter;

    public GetHoodReadyToHookCommandGroup(ClimberSubsystem climber, ShooterSubsystem shooter) {
        this.climber = climber;
        this.shooter = shooter;
        addCommands(
            //Lower climber partly to allow hood passage
            new ClimberToPointCommand(climber, CLIMBER_HEIGHT_TO_ALLOW_HOOD_PASSAGE),
            //Make hood ready to hook
            new SetHoodAngleCommand(shooter, HOOD_ANGLE_READY_TO_HOOK)
        );
    }
}
