package org.frcteam2910.c2022.util;

import edu.wpi.first.math.util.Units;
import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = Units.inchesToMeters(0.1);

    private final Trajectory fenderBluePartOne;
    private final Trajectory fenderRedPartOne;
    private final Trajectory twoBallWhitePartOne;
    private final Trajectory twoBallPurplePartOne;
    private final Trajectory fourBallOrangeAutoPartOne;
    private final Trajectory fourBallOrangeAutoPartTwo;
    private final Trajectory fourBallOrangeAutoPartThree;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) {
        fenderBluePartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(-0.2159, -1.21204), Rotation2.fromDegrees(69.0))
                        .arcTo(new Vector2(-0.15978, -2.02101), new Vector2(0.68081, -1.55626),
                                Rotation2.fromDegrees(69.0))
                        .lineTo(new Vector2(0.56977, -3.34051), Rotation2.fromDegrees(24.0)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        fenderRedPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(-1.04525, 0.6504), Rotation2.fromDegrees(-21.0))
                        .arcTo(new Vector2(-1.61166, 1.22155), new Vector2(-0.69418, 1.56498),
                                Rotation2.fromDegrees(-21.0))
                        .lineTo(new Vector2(-2.40481, 3.34051), Rotation2.fromDegrees(24.0)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        twoBallWhitePartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(-2.3062, 1.0836), Rotation2.fromDegrees(-43.5))
                        .arcTo(new Vector2(-4.11122, 2.00675), new Vector2(-2.35657, 3.21133),
                                Rotation2.fromDegrees(-32.13))
                        .arcTo(new Vector2(-3.76673, 2.36561), new Vector2(-3.90181, 2.15051)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        twoBallPurplePartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(-1.87974, -1.55069), Rotation2.fromDegrees(46.5))
                        .arcTo(new Vector2(-3.4527, -2.99959), new Vector2(-3.84217, -0.9985),
                                Rotation2.fromDegrees(34.88))
                        .arcTo(new Vector2(-3.64648, -2.5419), new Vector2(-3.50122, -2.75027)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        fourBallOrangeAutoPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(-0.72426, -2.44413), Rotation2.fromDegrees(91.5))
                        .lineTo(new Vector2(-0.65989, -3.27127))
                        .arcTo(new Vector2(-0.82907, -3.47542), new Vector2(-0.84981, -3.28605),
                                Rotation2.fromDegrees(91.5))
                        .arcTo(new Vector2(-3.97987, -2.7743), new Vector2(-1.3745, 1.50379),
                                Rotation2.fromDegrees(34.88))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        fourBallOrangeAutoPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(-3.97987, -2.7743), Rotation2.fromDegrees(34.88))
                        .lineTo(new Vector2(-3.64648, -2.5419))
                        .arcTo(new Vector2(-3.65731, -2.11801), new Vector2(-3.79173, -2.33353),
                                Rotation2.fromDegrees(34.88))
                        .arcTo(new Vector2(-6.53177, -2.415), new Vector2(-4.90345, -4.11597),
                                Rotation2.fromDegrees(-145.12))
                        .lineTo(new Vector2(-6.75195, -2.62578)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        fourBallOrangeAutoPartThree = new Trajectory(
                new SimplePathBuilder(new Vector2(-6.75195, -2.62578), Rotation2.fromDegrees(-145.12))
                        .lineTo(new Vector2(-3.97987, -2.7743), Rotation2.fromDegrees(34.88)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);
    }

    public Trajectory getFenderBluePartOne() {
        return fenderBluePartOne;
    }

    public Trajectory getFenderRedPartOne() {
        return fenderRedPartOne;
    }

    public Trajectory getTwoBallWhitePartOne() {
        return twoBallWhitePartOne;
    }

    public Trajectory getTwoBallPurplePartOne() {
        return twoBallPurplePartOne;
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
}
