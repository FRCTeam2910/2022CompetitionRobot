package org.frcteam2910.c2022.util;

import edu.wpi.first.math.geometry.Pose2d;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class Utilities {
    public static RigidTransform2 poseToRigidTransform(Pose2d pose){
        return new RigidTransform2(new Vector2(pose.getX(), pose.getY()), Rotation2.fromDegrees(pose.getRotation().getDegrees()));
    }
}
