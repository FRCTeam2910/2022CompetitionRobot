package org.frcteam2910.c2022.util;

import edu.wpi.first.math.util.Units;
import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = Units.inchesToMeters(0.1);

    private final Trajectory testAutoPartOne;
    private final Trajectory fenderBluePartOne;
    private final Trajectory fenderRedPartOne;
    private final Trajectory twoBallWhitePartOne;
    private final Trajectory twoBallPurplePartOne;
    private final Trajectory fourBallOrangeAutoPartOne;
    private final Trajectory fourBallOrangeAutoPartTwo;
    private final Trajectory fourBallOrangeAutoPartThree;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) {
        testAutoPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.fromDegrees(0.0))
                        .lineTo(new Vector2(Units.feetToMeters(12.0), 0.0)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        fenderBluePartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(-0.290, -1.245), Rotation2.fromDegrees(69.0))
                        .arcTo(new Vector2(-0.246, -1.926), new Vector2(0.475, -1.538), Rotation2.fromDegrees(69.0))
                        .lineTo(new Vector2(0.474, -3.265), Rotation2.fromDegrees(24.0)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        fenderRedPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(-1.119, 0.618), Rotation2.fromDegrees(-21.0))
                        .arcTo(new Vector2(-1.598, 1.096), new Vector2(-0.820, 1.396), Rotation2.fromDegrees(-21.0))
                        .lineTo(new Vector2(-2.434, 3.265), Rotation2.fromDegrees(24.0)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        twoBallWhitePartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(-2.386, 0.669), Rotation2.fromDegrees(1.5))
                        .arcTo(new Vector2(-4.356, 1.897), new Vector2(-2.327, 2.956), Rotation2.fromDegrees(-32.13))
                        .arcTo(new Vector2(-3.815, 2.396), new Vector2(-4.018, 2.073))
                        .lineTo(new Vector2(-3.294, 2.068)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        twoBallPurplePartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(-1.765, -1.581), Rotation2.fromDegrees(46.5))
                        .arcTo(new Vector2(-3.477, -3.268), new Vector2(-3.472, -1.561), Rotation2.fromDegrees(34.88))
                        .arcTo(new Vector2(-3.693, -2.575), new Vector2(-3.475, -2.887))
                        .lineTo(new Vector2(-3.188, -2.222)).build(),
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

    public Trajectory getTestAutoPartOne() {
        return testAutoPartOne;
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
