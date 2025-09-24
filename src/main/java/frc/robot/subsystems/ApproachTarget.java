// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.path.GoalEndState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

/** Reef target positions using blue side AprilTag IDs */
public enum ApproachTarget {
    
    UNKNOWN(0, 0, ApproachPosition.ALGAE, false),
    REEF_A(18, 7, ApproachPosition.LEFT, false),
    REEF_B(18, 7, ApproachPosition.RIGHT, false),
    REEF_AB(18, 7, ApproachPosition.ALGAE, false),
    REEF_C(17, 8, ApproachPosition.LEFT, false),
    REEF_D(17, 8, ApproachPosition.RIGHT, false),
    REEF_CD(17, 8, ApproachPosition.ALGAE, false),
    REEF_E(22, 9, ApproachPosition.LEFT, false),
    REEF_F(22, 9, ApproachPosition.RIGHT, false),
    REEF_EF(22, 9, ApproachPosition.ALGAE, false),
    REEF_G(21, 10, ApproachPosition.LEFT, false),
    REEF_H(21, 10, ApproachPosition.RIGHT, false),
    REEF_GH(21, 10, ApproachPosition.ALGAE, false),
    REEF_I(20, 11, ApproachPosition.LEFT, false),
    REEF_J(20, 11, ApproachPosition.RIGHT, false),
    REEF_IJ(20, 11, ApproachPosition.ALGAE, false),
    REEF_K(19, 6, ApproachPosition.LEFT, false),
    REEF_L(19, 6, ApproachPosition.RIGHT, false),
    REEF_KL(19, 6, ApproachPosition.ALGAE, false),
    PROCESSOR(16, 3, ApproachPosition.PROCESSOR, false),
    BARGE(14, 5, ApproachPosition.OVERHEAD, true),
    LEFT_SOURCE(13, 1, ApproachPosition.ALGAE, true),
    RIGHT_SOURCE(12, 2, ApproachPosition.ALGAE, true);
    
    public final int blueTagId;
    public final int redTagId;
    public final boolean enableHighCam;

    public final ApproachPosition position;

    public final Pose2d blueTagPose;
    public final Pose2d bluePt1;
    public final Pose2d bluePt2;
    public final GoalEndState blueGoalEndState;

    public final Pose2d redTagPose;
    public final Pose2d redPt1;
    public final Pose2d redPt2;
    public final GoalEndState redGoalEndState;

    private ApproachTarget(int blueTagId, int redTagId, ApproachPosition position, boolean enableHighCam){

        // System.err.println("AprilTags Initialized");

        this.blueTagId = blueTagId;
        this.redTagId = redTagId;
        this.enableHighCam = enableHighCam;

        this.position = position;

        this.blueTagPose = (blueTagId == 0) ? Pose2d.kZero : Constants.kFieldLayout.getTagPose(blueTagId).get().toPose2d();
        this.bluePt1 = blueTagPose.plus(position.pt1Transform);
        this.bluePt2 = blueTagPose.plus(position.pt2Transform);
        this.blueGoalEndState = new GoalEndState(0.0, (position == ApproachPosition.OVERHEAD) ? blueTagPose.getRotation() : blueTagPose.getRotation().rotateBy(Rotation2d.k180deg));

        this.redTagPose = (redTagId == 0) ? Pose2d.kZero : Constants.kFieldLayout.getTagPose(redTagId).get().toPose2d();
        this.redPt1 = redTagPose.plus(position.pt1Transform);
        this.redPt2 = redTagPose.plus(position.pt2Transform);
        this.redGoalEndState = new GoalEndState(0.0, (position == ApproachPosition.OVERHEAD) ? redTagPose.getRotation() : redTagPose.getRotation().rotateBy(Rotation2d.k180deg));
    }
}
