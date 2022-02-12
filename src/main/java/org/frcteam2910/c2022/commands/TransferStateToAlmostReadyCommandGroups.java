package org.frcteam2910.c2022.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frcteam2910.c2022.subsystems.ClimberSubsystem;
import org.frcteam2910.c2022.subsystems.ShooterSubsystem;

public class TransferStateToAlmostReadyCommandGroups extends ParallelCommandGroup {

    public static final double CLIMBER_HEIGHT_TO_ALLOW_HOOD_PASSAGE = 0.5;
    public static final double HOOD_ANGLE_ALMOST_READY_TO_HOOK = Math.toRadians(70);

    public final ShooterSubsystem shooter;
    public final ClimberSubsystem climber;

    public TransferStateToAlmostReadyCommandGroups(ClimberSubsystem climber, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.climber = climber;
        addCommands(
                //Lower climber partly to allow hood passage
                new ClimberToPointCommand(climber, CLIMBER_HEIGHT_TO_ALLOW_HOOD_PASSAGE),
                //Make hood ready to hook
                new WaitCommand(0.5).andThen(new SetHoodAngleCommand(shooter, HOOD_ANGLE_ALMOST_READY_TO_HOOK))
        );
    }
};
