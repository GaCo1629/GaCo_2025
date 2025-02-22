package frc.robot.subsystems;

public enum TowerState {
	//Start
	INIT,
	HOME,

	//Scoring
	INTAKING,
	HAVE_CORAL,
	RAISING_AND_TILTING, //L1 L2
	GOING_TO_SAFE, // L3 L4
	RAISING, // L3 L4
	TILTING_TO_SCORE, // L3 L4
	READY_TO_SCORE,
	SCORING_CORAL,

	//Returning
	LOWERING_AND_TILTING, // L1 L2
	RETURNING_TO_SAFE, // L3 L4
	LOWERING, // L3 L4
	TILTING_TO_HOME // L3 L4
}
