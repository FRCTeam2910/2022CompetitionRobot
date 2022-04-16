package org.frcteam2910.c2022.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.frcteam2910.c2022.lib.wpilib.InterpolatingTreeMap;

public class TargetingSolver {
    private static final int REFINEMENT_ITERATIONS = 20;
    private static final double MIN_REFINEMENT_DISTANCE_CHANGE = 0.01;

    private final InterpolatingTreeMap<Double, TargetingLookupEntry> lookupMap = new InterpolatingTreeMap<>();

    private final Translation2d targetPosition;

    public TargetingSolver(Translation2d targetPosition) {
        this.targetPosition = targetPosition;
    }

    public void addEntry(double distance, TargetingLookupEntry entry) {
        lookupMap.put(distance, entry);
    }

    private TargetingSolution initial(Translation2d currentPosition, Translation2d currentVelocity) {
        Translation2d delta = targetPosition.minus(currentPosition);
        TargetingLookupEntry entry = lookupMap.get(delta.getNorm());

        return new TargetingSolution(entry.hoodAngleRadians, entry.shooterVelocityRadiansPerSecond,
                entry.airTimeSeconds, new Rotation2d(delta.getX(), delta.getY()), targetPosition);
    }

    public TargetingSolution solve(Translation2d currentPosition, Translation2d currentVelocity) {
        // Get initial guess
        TargetingSolution solution = initial(currentPosition, currentVelocity);

        // Refine the guess. The more times we refine the guess, the closer it gets to
        // the actual solution.
        // Typically, we converge onto a solution in <5 iterations. We set a higher
        // iteration count, but have an
        // additional check to determine how much the solution changed. If the solution
        // didn't change much, we'll exit
        // the loop early.
        for (int i = 0; i < REFINEMENT_ITERATIONS; i++) {
            TargetingSolution refinedSolution = refine(solution, currentPosition, currentVelocity);

            double refinementDistanceChanged = refinedSolution.targetPosition.minus(solution.targetPosition).getNorm();
            solution = refinedSolution;

            if (refinementDistanceChanged < MIN_REFINEMENT_DISTANCE_CHANGE) {
                break;
            }
        }

        return solution;
    }

    private TargetingSolution refine(TargetingSolution previous, Translation2d currentPosition,
            Translation2d currentVelocity) {
        // Calculate the position we should be targeting based on the current velocity
        // and the amount of time the
        // previous solution's shot is in the air.
        Translation2d targetPosition = this.targetPosition.plus(currentVelocity.times(-previous.estimatedAirTime));

        // Calculate how far away the current position is from the target. This distance
        // is what we'll use to determine
        // the shooter speed and angle
        Translation2d delta = targetPosition.minus(currentPosition);
        TargetingLookupEntry entry = lookupMap.get(delta.getNorm());

        return new TargetingSolution(entry.hoodAngleRadians, entry.shooterVelocityRadiansPerSecond,
                entry.airTimeSeconds, new Rotation2d(delta.getX(), delta.getY()), targetPosition);
    }

    public Translation2d getTargetPosition() {
        return targetPosition;
    }
}
