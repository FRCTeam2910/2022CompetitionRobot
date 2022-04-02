package org.frcteam2910.c2022.util;

import edu.wpi.first.math.util.Units;
import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.control.TrajectoryConstraint;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = Units.inchesToMeters(0.1);

    private final Trajectory testAutoPartOne;
    private final Trajectory fenderBluePartOne;
    private final Trajectory fenderRedPartOne;
    private final Trajectory twoBallGreenPartOne;
    private final Trajectory twoBallPurplePartOne;
    private final Trajectory threeBallOrangePartOne;
    private final Trajectory threeBallOrangePartTwo;
    private final Trajectory threeBallOrangePartThree;
    private final Trajectory fiveBallOrangePartOne;
    private final Trajectory fiveBallOrangePartTwo;
    private final Trajectory sixBallOrangePartOne;
    private final Trajectory sixBallOrangePartTwo;
    private final Trajectory sixBallOrangePartThree;
    private final Trajectory sixBallOrangePartFour;
    private final Trajectory sixBallOrangePartFive;
    private final Trajectory sixBallOrangePartSix;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) {
        testAutoPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.fromDegrees(0.0))
                        .lineTo(new Vector2(Units.feetToMeters(30.0), 0.0)).build(),
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

        twoBallGreenPartOne = new Trajectory(
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

        threeBallOrangePartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(-0.669, -2.386), Rotation2.fromDegrees(-88.5))
                        .lineTo(new Vector2(-0.66, -3.417), Rotation2.fromDegrees(-90.0)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        threeBallOrangePartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(-0.66, -3.417), Rotation2.fromDegrees(-90.0))
                        .lineTo(new Vector2(-3.831, -3.197), Rotation2.fromDegrees(34.88))
                        .arcTo(new Vector2(-3.937, -2.744), new Vector2(-3.794, -2.949)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        threeBallOrangePartThree = new Trajectory(
                new SimplePathBuilder(new Vector2(-3.937, -2.744), Rotation2.fromDegrees(34.88))
                        .lineTo(new Vector2(-3.527, -2.458)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        fiveBallOrangePartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(-3.527, -2.458), Rotation2.fromDegrees(34.88))
                        .lineTo(new Vector2(-6.431, -2.409), Rotation2.fromDegrees(-136.25))
                        .arcTo(new Vector2(-6.963, -2.617), new Vector2(-6.444, -3.158)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        fiveBallOrangePartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(-6.963, -2.617), Rotation2.fromDegrees(-136.25))
                        .lineTo(new Vector2(-3.527, -2.458), Rotation2.fromDegrees(34.88)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        sixBallOrangePartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(-0.669, -2.386), Rotation2.fromDegrees(-88.5))
                        .lineTo(new Vector2(-0.66, -3.417), Rotation2.fromDegrees(-90.0)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        sixBallOrangePartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(-0.66, -3.417), Rotation2.fromDegrees(-90.0))
                        .lineTo(new Vector2(-3.2, -3.284))
                        .arcTo(new Vector2(-3.95, -2.6), new Vector2(-3.158, -2.485), Rotation2.fromDegrees(34.88))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        sixBallOrangePartThree = new Trajectory(
                new SimplePathBuilder(new Vector2(-3.95, -2.6), Rotation2.fromDegrees(34.88))
                        .lineTo(new Vector2(-3.54, -2.314)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        sixBallOrangePartFour = new Trajectory(
                new SimplePathBuilder(new Vector2(-3.54, -2.314), Rotation2.fromDegrees(34.88))
                        .lineTo(new Vector2(-6.963, -2.617), Rotation2.fromDegrees(-136.25)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        sixBallOrangePartFive = new Trajectory(
                new SimplePathBuilder(new Vector2(-6.963, -2.617), Rotation2.fromDegrees(-136.25))
                        .lineTo(new Vector2(-7.059, 0.138))
                        .arcTo(new Vector2(-3.828, 2.615), new Vector2(-4.561, 0.225), Rotation2.fromDegrees(-34.34))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        sixBallOrangePartSix = new Trajectory(
                new SimplePathBuilder(new Vector2(-3.828, 2.615), Rotation2.fromDegrees(-34.34))
                        .lineTo(new Vector2(-3.209, 2.192)).build(),
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

    public Trajectory getTwoBallGreenPartOne() {
        return twoBallGreenPartOne;
    }

    public Trajectory getTwoBallPurplePartOne() {
        return twoBallPurplePartOne;
    }

    public Trajectory getThreeBallOrangePartOne() {
        return threeBallOrangePartOne;
    }

    public Trajectory getThreeBallOrangePartTwo() {
        return threeBallOrangePartTwo;
    }

    public Trajectory getThreeBallOrangePartThree() {
        return threeBallOrangePartThree;
    }

    public Trajectory getFiveBallOrangePartOne() {
        return fiveBallOrangePartOne;
    }

    public Trajectory getFiveBallOrangePartTwo() {
        return fiveBallOrangePartTwo;
    }

    public Trajectory getSixBallOrangePartOne() {
        return sixBallOrangePartOne;
    }

    public Trajectory getSixBallOrangePartTwo() {
        return sixBallOrangePartTwo;
    }

    public Trajectory getSixBallOrangePartThree() {
        return sixBallOrangePartThree;
    }

    public Trajectory getSixBallOrangePartFour() {
        return sixBallOrangePartFour;
    }

    public Trajectory getSixBallOrangePartFive() {
        return sixBallOrangePartFive;
    }

    public Trajectory getSixBallOrangePartSix() {
        return sixBallOrangePartSix;
    }
}
