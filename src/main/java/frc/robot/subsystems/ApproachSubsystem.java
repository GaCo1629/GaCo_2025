// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import frc.robot.Constants.ApproachConstants;

import java.util.List;
import java.util.Optional;

public class ApproachSubsystem extends SubsystemBase {

	private CommandScheduler scheduler = CommandScheduler.getInstance();
	public AprilTagFieldLayout tags =
			AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded); // CHS uses andymark, worlds uses welded
	public Optional<Alliance> alliance = DriverStation.getAlliance();
	private CommandSwerveDrivetrain drivetrain;
	private PathPlannerPath path;

	public ApproachSubsystem(CommandSwerveDrivetrain drivetrain) {
		this.drivetrain = drivetrain;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
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

	/* Create a Path Command to navigate to the specified position **/
	public Command buildPathCmd(ApproachTarget targetPos) {
		// Get tag coordinates and heading
		int adjTagID = getTagId(targetPos.tagId);
		Pose2d tag = tags.getTagPose(adjTagID).get().toPose2d();

		SmartDashboard.putString(
				"Tag Info",
				String.format(
						"ID%d X:%.3f Y:%.3f T:%.1f",
						adjTagID, tag.getX(), tag.getY(), tag.getRotation().getRadians()));

		// Create a list of three waypoints.
		// The rotation component of the pose should be the direction of travel. Do not use
		// holonomic rotation.
		List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
				drivetrain.getState().Pose,
				tag.plus(targetPos.position.pt1Transform),
				tag.plus(targetPos.position.pt2Transform));

		// limit severity of motion.
		PathConstraints constraints = new PathConstraints(
				ApproachConstants.maxApproachLinearVelocity,
				ApproachConstants.maxApproachLinearAcceleration,
				ApproachConstants.maxApproachAngularVelocity,
				ApproachConstants.maxApproachAngularAcceleration);

		// Create and return the path using the waypoints created above
		path = new PathPlannerPath(
				waypoints,
				constraints,
				null,
				new GoalEndState(
						0.0,
						(targetPos.position == ApproachPosition.OVERHEAD)
								? tag.getRotation()
								: tag.getRotation().rotateBy(Rotation2d.k180deg)) // Goal end state.
				);

		path.preventFlipping = true;
		return AutoBuilder.followPath(path);
	}

	// Modify tag ID is running on Red Alliance.
	private int getTagId(int id) {
		alliance = DriverStation.getAlliance();
		if (alliance.isPresent() && alliance.get().equals(Alliance.Red)) {
			switch (id) {
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