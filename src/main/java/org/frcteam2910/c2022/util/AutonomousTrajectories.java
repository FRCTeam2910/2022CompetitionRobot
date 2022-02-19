package org.frcteam2910.c2022.util;

import java.util.Arrays;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;

    private final Trajectory oneBallAutoPartOne;
    private final Trajectory threeBallAutoPartOne;
    private final Trajectory threeBallAutoPartTwo;
    private final Trajectory threeBallAutoPartThree;
    private final Trajectory fourBallAutoPartOne;
    private final Trajectory fourBallAutoPartTwo;
    private final Trajectory fourBallAutoPartThree;
    private final Trajectory fourBallAutoPartFour;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);

        oneBallAutoPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO).lineTo(new Vector2(-50.0, 0.0)).build(),
                slowConstraints, SAMPLE_DISTANCE);

        threeBallAutoPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO).lineTo(new Vector2(0.0, -10.0)).build(),
                slowConstraints, SAMPLE_DISTANCE);

        threeBallAutoPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, -10.0), Rotation2.ZERO).lineTo(new Vector2(-20.0, 5.0)).build(),
                slowConstraints, SAMPLE_DISTANCE);

        threeBallAutoPartThree = new Trajectory(
                new SimplePathBuilder(new Vector2(-20.0, 5.0), Rotation2.ZERO).lineTo(new Vector2(-18.0, 6.0)).build(),
                slowConstraints, SAMPLE_DISTANCE);

        fourBallAutoPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO).lineTo(new Vector2(0.0, -10.0)).build(),
                slowConstraints, SAMPLE_DISTANCE);

        fourBallAutoPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, -10), Rotation2.ZERO).lineTo(new Vector2(-18.0, 6.0)).build(),
                slowConstraints, SAMPLE_DISTANCE);

        fourBallAutoPartThree = new Trajectory(
                new SimplePathBuilder(new Vector2(-18.0, 6.0), Rotation2.ZERO).lineTo(new Vector2(-100.0, 0.0)).build(),
                slowConstraints, SAMPLE_DISTANCE);

        fourBallAutoPartFour = new Trajectory(
                new SimplePathBuilder(new Vector2(-100.0, 0.0), Rotation2.ZERO).lineTo(new Vector2(-18.0, 6.0)).build(),
                slowConstraints, SAMPLE_DISTANCE);
    }

    public Trajectory getOneBallAutoPartOne() {
        return oneBallAutoPartOne;
    }

    public Trajectory getThreeBallAutoPartOne() {
        return threeBallAutoPartOne;
    }

    public Trajectory getThreeBallAutoPartTwo() {
        return threeBallAutoPartTwo;
    }

    public Trajectory getThreeBallAutoPartThree() {
        return threeBallAutoPartThree;
    }

    public Trajectory getFourBallAutoPartOne() {
        return fourBallAutoPartOne;
    }

    public Trajectory getFourBallAutoPartTwo() {
        return fourBallAutoPartTwo;
    }

    public Trajectory getFourBallAutoPartThree() {
        return fourBallAutoPartThree;
    }

    public Trajectory getFourBallAutoPartFour() {
        return fourBallAutoPartFour;
    }
}
