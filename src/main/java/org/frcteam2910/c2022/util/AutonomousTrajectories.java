package org.frcteam2910.c2022.util;

import java.util.Arrays;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;

    private final Trajectory fenderBluePartOne;
    private final Trajectory fenderRedPartOne;
    private final Trajectory twoBallWhitePartOne;
    private final Trajectory twoBallPurplePartOne;
    private final Trajectory threeBallOrangeAutoPartOne;
    private final Trajectory threeBallOrangeAutoPartTwo;
    private final Trajectory threeBallOrangeAutoPartThree;
    private final Trajectory fourBallOrangeAutoPartOne;
    private final Trajectory fourBallOrangeAutoPartTwo;
    private final Trajectory fourBallOrangeAutoPartThree;
    private final Trajectory fourBallOrangeAutoPartFour;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);

        fenderBluePartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO).lineTo(new Vector2(0.0, 0.0)).build(),
                slowConstraints, SAMPLE_DISTANCE);

        fenderRedPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO).lineTo(new Vector2(0.0, 0.0)).build(),
                slowConstraints, SAMPLE_DISTANCE);

        twoBallWhitePartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0,0.0), Rotation2.ZERO)
                    .lineTo(new Vector2(0.0,0.0))
                    .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        twoBallPurplePartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0,0.0), Rotation2.ZERO)
                .lineTo(new Vector2(0.0,0.0))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeBallOrangeAutoPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO).lineTo(new Vector2(0.0, 0.0)).build(),
                slowConstraints, SAMPLE_DISTANCE);

        threeBallOrangeAutoPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(0.0, 0.0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeBallOrangeAutoPartThree = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(0.0, 0.0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        fourBallOrangeAutoPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(0.0, 0.0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        fourBallOrangeAutoPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, -10), Rotation2.ZERO).lineTo(new Vector2(8.0, 0.0)).build(),
                slowConstraints, SAMPLE_DISTANCE);

        fourBallOrangeAutoPartThree = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO).lineTo(new Vector2(0.0, 0.0)).build(),
                slowConstraints, SAMPLE_DISTANCE);

        fourBallOrangeAutoPartFour = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO).lineTo(new Vector2(0.0, 0.0)).build(),
                slowConstraints, SAMPLE_DISTANCE);
    }

    public Trajectory getFenderBluePartOne() {
        return fenderBluePartOne;
    }

    public Trajectory getFenderRedPartOne() {
        return fenderRedPartOne;
    }

    public Trajectory getTwoBallWhitePartOne(){
        return twoBallWhitePartOne;
    }

    public Trajectory getTwoBallPurplePartOne(){
        return twoBallWhitePartOne;
    }

    public Trajectory getThreeBallOrangeAutoPartOne() {
        return threeBallOrangeAutoPartOne;
    }

    public Trajectory getThreeBallOrangeAutoPartTwo() {
        return threeBallOrangeAutoPartTwo;
    }

    public Trajectory getThreeBallOrangeAutoPartThree() {
        return threeBallOrangeAutoPartThree;
    }

    public Trajectory getFourBallOrangeAutoPartOne() {
        return fourBallOrangeAutoPartOne;
    }

    public Trajectory getFourBallOrangeAutoPartTwo() {
        return fourBallOrangeAutoPartTwo;
    }

    public Trajectory getFourBallOrangeAutoPartThree() {
        return fourBallOrangeAutoPartThree;
    }

    public Trajectory getFourBallOrangeAutoPartFour() {
        return fourBallOrangeAutoPartFour;
    }
}
