// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.Optional;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ApproachConstants;

public class ApproachSubsystem extends SubsystemBase {

  private CommandScheduler scheduler = CommandScheduler.getInstance();
//  public AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark); //CHS uses andymark, worlds uses welded
  public AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded); //CHS uses andymark, worlds uses welded
  public Optional<Alliance> alliance = DriverStation.getAlliance();
  private CommandSwerveDrivetrain drivetrain;
  private PathPlannerPath path;
  
  public ApproachSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Approach Target", Globals.IDENTIFIED_TARGET.toString());
  }

  public void identifyTarget(ApproachTarget targetPos) {
    Globals.IDENTIFIED_TARGET = targetPos;
  }

  public void startApproach() {
    Globals.setLEDMode(LEDmode.APPROACH);
    if (Globals.IDENTIFIED_TARGET != ApproachTarget.UNKNOWN) {
      scheduler.schedule(buildPathCmd(Globals.IDENTIFIED_TARGET));
    }
  }

  static final Distance BUMPER_TO_CENTER = Meters.of(0.45);
  static final Distance REEF_HALF_WIDTH  = Meters.of(0.165);       // Offset from the center to the pole
  static final Distance NORMAL_APPROACH_DISTANCE = Meters.of(0.2); // Target Distance from reef when first approaching
  static final Distance ALGAE_STANDOFF   = Meters.of(0.80);        // Extra distance for wrist fold down
  static final Distance OVERHEAD_STANDOFF = Meters.of(0.0);        // Negative is more under the barge
  
  /* Create a Path Command to navigate to the specified position **/
  public Command buildPathCmd(ApproachTarget targetPos){
    // Distance in meters
    MutDistance centerStandoff   = BUMPER_TO_CENTER.mutableCopy();
    MutDistance reefBranchOffset = REEF_HALF_WIDTH.mutableCopy();      

    // Get tag coordinates and heading
    int adjTagID    = getTagId(targetPos.tagId);
    //Pose2d tag = tags.getTagPose(adjTagID).get().toPose2d();
    //Distance tagX     = tags.getTagPose(adjTagID).get().getMeasureX();
    //Distance tagY     = tags.getTagPose(adjTagID).get().getMeasureY();
    //Rotation2d tagAngle = tags.getTagPose(adjTagID).get().toPose2d().getRotation();
    //Distance offsetX  = Meters.of(0.0);
    //Distance offsetY  = Meters.of(0.0);    

    SmartDashboard.putString("Tag Info", String.format("ID%d X:%.3f Y:%.3f T:%.1f", adjTagID, tags.getTagPose(adjTagID).get().toPose2d().getX(), tags.getTagPose(adjTagID).get().toPose2d().getY(), tags.getTagPose(adjTagID).get().toPose2d().getRotation().getRadians()));
    
    // adjust offset and standoff based on specific target location
    if(targetPos.position == ApproachPosition.LEFT){
      reefBranchOffset.mut_times(-1.0);
    } else if(targetPos.position == ApproachPosition.ALGAE){
      reefBranchOffset.mut_replace(0.0, Meters);
      centerStandoff.mut_replace(ALGAE_STANDOFF); // Space out further for algae
    } else if(targetPos.position == ApproachPosition.OVERHEAD){
      reefBranchOffset.mut_replace(0.4, Meters);
      centerStandoff.mut_replace(OVERHEAD_STANDOFF); // Space out further for algae
    }
    Globals.HIGH_CAM_ENABLED = targetPos.enableHighCam;

    // Calculate left/right offsets for branch coordinates
    Translation2d offset = (reefBranchOffset.isEquivalent(OVERHEAD_STANDOFF)) ? Translation2d.kZero : new Translation2d(reefBranchOffset.in(Meters), tags.getTagPose(adjTagID).get().toPose2d().getRotation());
    //if (reefBranchOffset.isEquivalent(OVERHEAD_STANDOFF)) {
      //offset = new Translation2d(reefBranchOffset.in(Meters), tags.getTagPose(adjTagID).get().toPose2d().getRotation());
      //offsetX  = reefBranchOffset.times(Math.cos(tagAngle.in(Radians) + Math.PI/2));
      //offsetY  = reefBranchOffset.times(Math.sin(tagAngle.in(Radians) + Math.PI/2));
    //}

    //Rotation2d finalAngle = tags.getTagPose(adjTagID).get().toPose2d().getRotation().rotateBy(Rotation2d.k180deg);

    // Determine intermediate and final approach points
    //Translation2d pt0 = drivetrain.getState().Pose.getTranslation();
    //Pose2d pt0 = drivetrain.getState().Pose;
    //Translation2d pt1 = new Translation2d(tag.getX() + tagAngle.getCos() * centerStandoff.in(Meters) + NORMAL_APPROACH_DISTANCE.in(Meters) + offset.getX(), tag.getY() + tagAngle.getSin() * centerStandoff.in(Meters) + NORMAL_APPROACH_DISTANCE.in(Meters) + offset.getY());
    //Pose2d pt1 = new Pose2d(tags.getTagPose(adjTagID).get().toPose2d().getX() + tags.getTagPose(adjTagID).get().toPose2d().getRotation().getCos() * centerStandoff.in(Meters) + NORMAL_APPROACH_DISTANCE.in(Meters) + offset.getX(), tags.getTagPose(adjTagID).get().toPose2d().getY() + tags.getTagPose(adjTagID).get().toPose2d().getRotation().getSin() * centerStandoff.in(Meters) + NORMAL_APPROACH_DISTANCE.in(Meters) + offset.getY(), tags.getTagPose(adjTagID).get().toPose2d().getRotation().rotateBy(Rotation2d.k180deg));
    //Translation2d pt2 = new Translation2d(tag.getX() + tagAngle.getCos() * centerStandoff.in(Meters) + offset.getX(), tag.getY() + tagAngle.getSin() * centerStandoff.in(Meters) + offset.getY());
    //Pose2d pt2 = new Pose2d(tags.getTagPose(adjTagID).get().toPose2d().getX() + tags.getTagPose(adjTagID).get().toPose2d().getRotation().getCos() * centerStandoff.in(Meters) + offset.getX(), tags.getTagPose(adjTagID).get().toPose2d().getY() + tags.getTagPose(adjTagID).get().toPose2d().getRotation().getSin() * centerStandoff.in(Meters) + offset.getY(), tags.getTagPose(adjTagID).get().toPose2d().getRotation().rotateBy(Rotation2d.k180deg));

    // determine trajectory angles for starting and ending path.
    //Rotation2d initialAngle  = Rotation2d.fromRadians(Math.atan2(pt1.getY() - pt0.getY(), pt1.getY() - pt0.getX()));
    //Rotation2d overheadAngle = tag.getRotation();

    // Create a list of three waypoints.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      drivetrain.getState().Pose,
      new Pose2d(tags.getTagPose(adjTagID).get().toPose2d().getX() + tags.getTagPose(adjTagID).get().toPose2d().getRotation().getCos() * centerStandoff.in(Meters) + NORMAL_APPROACH_DISTANCE.in(Meters) + offset.getX(), tags.getTagPose(adjTagID).get().toPose2d().getY() + tags.getTagPose(adjTagID).get().toPose2d().getRotation().getSin() * centerStandoff.in(Meters) + NORMAL_APPROACH_DISTANCE.in(Meters) + offset.getY(), tags.getTagPose(adjTagID).get().toPose2d().getRotation().rotateBy(Rotation2d.k180deg)),
      new Pose2d(tags.getTagPose(adjTagID).get().toPose2d().getX() + tags.getTagPose(adjTagID).get().toPose2d().getRotation().getCos() * centerStandoff.in(Meters) + offset.getX(), tags.getTagPose(adjTagID).get().toPose2d().getY() + tags.getTagPose(adjTagID).get().toPose2d().getRotation().getSin() * centerStandoff.in(Meters) + offset.getY(), tags.getTagPose(adjTagID).get().toPose2d().getRotation().rotateBy(Rotation2d.k180deg))
    );

    // limit severity of motion.
    PathConstraints constraints = new PathConstraints(ApproachConstants.maxApproachLinearVelocity, ApproachConstants.maxApproachLinearAcceleration, ApproachConstants.maxApproachAngularVelocity, ApproachConstants.maxApproachAngularAcceleration); 
    
    // Create and return the path using the waypoints created above
    path = new PathPlannerPath(
        waypoints,
        constraints,
        null,
        new GoalEndState(0.0, (targetPos.position == ApproachPosition.OVERHEAD) ? tags.getTagPose(adjTagID).get().toPose2d().getRotation() : tags.getTagPose(adjTagID).get().toPose2d().getRotation().rotateBy(Rotation2d.k180deg)) // Goal end state. 
    );

    path.preventFlipping = true;
    return AutoBuilder.followPath(path);
  }

  // Modify tag ID is running on Red Alliance.
  private int getTagId(int id){
    alliance = DriverStation.getAlliance();
    if(alliance.isPresent() && alliance.get().equals(Alliance.Red)){
      switch(id){
        case 12:
          id = 2;
          break;

        case 13:
          id = 1;
          break;

        case 14:
          id = 5;
          break;

        case 16:
          id = 3;
          break;

        case 17:
          id = 8;
          break;

        case 18:
          id = 7;
          break;

        case 19:
          id = 6;
          break;

        case 20:
          id = 11;
          break;

        case 21:
          id = 10;
          break;
          
        case 22:
          id = 9;
          break;
      }
    }

    return id;
  }
}
