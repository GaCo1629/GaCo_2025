package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Meters;

public enum ApproachPosition {
	LEFT(Meters.of(0.475), Meters.of(-0.165)),	// first Meter.of was 0.45
	RIGHT(Meters.of(0.475), Meters.of(0.165)),	// first Meter.of was 0.45
	ALGAE(Meters.of(0.80), Meters.of(0.0)),
	OVERHEAD(Meters.of(0.05), Meters.of(0.4));  // back of robot on Barge line	// first Meter.of was 0.0
	public final Transform2d pt1Transform;
	public final Transform2d pt2Transform;

	private final Distance NORMAL_APPROACH_DISTANCE = Meters.of(0.2);

	// Reef Branch offset is the offset of the center of the robot from the center of the branches.
	// Center Standoff is the distance away from the Reef the robot needs to be
	private ApproachPosition(Distance centerOffsetX, Distance centerOffsetY) {
		pt1Transform = new Transform2d(NORMAL_APPROACH_DISTANCE.plus(centerOffsetX), centerOffsetY, Rotation2d.k180deg);
		// Pt2 is against the reef, offset by the given reefBranchoffset.
		pt2Transform = new Transform2d(centerOffsetX, centerOffsetY, Rotation2d.k180deg);
	}
}