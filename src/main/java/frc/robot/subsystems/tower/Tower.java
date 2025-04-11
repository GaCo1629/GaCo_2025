// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tower;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.Driver;
import frc.robot.subsystems.Globals;
import frc.robot.subsystems.approach.ApproachTarget;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class Tower extends SubsystemBase {
	@AutoLogOutput
	private TowerState currentState = TowerState.INIT;

	private final Timer stateTimer = new Timer();

	private final Elevator elevator;
	private final Wrist wrist;
	private final CommandXboxController joystick;

	@AutoLogOutput
	private TowerEvent pendingEvent = TowerEvent.NONE; 

	@AutoLogOutput
	private double safetyFactor = 1;

	@AutoLogOutput
	private int currentLevel = 0;

	@AutoLogOutput
	private int goDirectAlgaeLevel = 0;

	/** Creates a new Tower. */
	public Tower(Elevator elevator, Wrist wrist, CommandXboxController joystick) {
		this.elevator = elevator;
		this.wrist = wrist;
		this.joystick = joystick;
		stateTimer.start();
	}

	public void initialize() {
		setState(TowerState.INIT);
		pendingEvent = TowerEvent.NONE;
		Globals.GOT_ALGAE = false;
		wrist.initialize();
		elevator.initialize();
	}

	public void resetFrameRate() {
		elevator.resetFrameRate();
		wrist.resetFrameRate();
	}

	public void setDirectAlgaeLevel(int level){
		goDirectAlgaeLevel = level;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		runStateMachine();
		updateDashboard();
	}

	public void homeTower() {
		wrist.setIntakeSpeed(0);
		joystick.setRumble(RumbleType.kBothRumble,0);
		elevator.resetElevatorControl();
		wrist.resetWristControl();
		pendingEvent = TowerEvent.NONE;
		setState(TowerState.INIT);
	}
	
	public void forceWristTilt(double angleDeg){
		wrist.setGoalAngle(angleDeg);
		wrist.setIntakeSpeed(0);
		setState(TowerState.GOING_TO_SAFE);
	}

	public void runStateMachine() {
		switch(currentState){
			// =================== Initializing ===================
			case INIT: {
				if (elevator.getHeightMeters() < Constants.Elevator.kSafeHomeHeightMeters) {
					elevator.setGoalPositionMeters(Constants.Elevator.kIntakeHeightMeters);	
					setState(TowerState.HOMING_ELEVATOR);
				} else {
					wrist.setGoalAngle(Constants.Wrist.kSafeAngleDegrees);
					setState(TowerState.MAKING_WRIST_SAFE);
				}
				break;
			}

			case MAKING_WRIST_SAFE: {
				if (wrist.inPosition()) {
					elevator.setGoalPositionMeters(Constants.Elevator.kIntakeHeightMeters);	
					setState(TowerState.HOMING_ELEVATOR);
				}
				break;
			}

			case HOMING_ELEVATOR: {
				if (elevator.inPosition()) {
					wrist.setGoalAngle(Constants.Wrist.kIntakeAngleDegrees);
					setState(TowerState.HOMING_WRIST);
				} else if (DriverStation.getStickButton(1, Driver.reset)) {
					elevator.resetEncoder();
				}
				break;
			}

			case HOMING_WRIST: {
				if (wrist.inPosition()){
					setState(TowerState.HOME);
				}
				break;
			}

			// =================== GENERIC HOME condition ===================
			
			case HOME: {
				if (isTriggered(TowerEvent.INTAKE_CORAL) || isHoldingGoTo()){ // will triger on L1-4 as well
					if (!wrist.gotExitCoral()) {  // this will be true if we are holding a coral
						wrist.setIntakeSpeed(Constants.Wrist.kCoralIntakePower);
					}
					setState(TowerState.INTAKING);
				} else if (isTriggered(TowerEvent.INTAKE_LOW_ALGAE)){
					wrist.setGoalAngle(Constants.Wrist.kSafeAngleDegrees);
					setState(TowerState.GET_SAFE_FOR_L2_ALGAE);
				} else if (isTriggered(TowerEvent.INTAKE_HIGH_ALGAE)){
					wrist.setGoalAngle(Constants.Wrist.kSafeAngleDegrees);
					setState(TowerState.GET_SAFE_FOR_L3_ALGAE);
				} else if (isTriggered(TowerEvent.SCORE)){
					wrist.setIntakeSpeed(Constants.Wrist.kCoralIntakePower);
					setState(TowerState.FINISHING_SCORING_CORAL);
				} else {
					currentLevel = 0;
					joystick.setRumble(RumbleType.kBothRumble,0);
				}
				break;
			}

			// ====  CORAL specific states  ========================================

			case INTAKING: {
				if (wrist.gotExitCoral()) {   // this will be true if we are holding a coral
					wrist.setIntakeSpeed(Constants.Wrist.kCoralHoldPower);
					joystick.setRumble(RumbleType.kBothRumble,1);
					setState(TowerState.INTAKE_PAUSE);
				} else if (wrist.gotEnterCoral()){
					wrist.setIntakeSpeed(Constants.Wrist.kCoralSlowIntakePower);
					joystick.setRumble(RumbleType.kBothRumble,1);
					setState(TowerState.INTAKE_PAUSE);
				}
				break;
			}

			case INTAKE_PAUSE: {
				if (wrist.gotExitCoral()) {
					wrist.setIntakeSpeed(Constants.Wrist.kCoralHoldPower);
				}
				
				if (stateTimer.hasElapsed(0.2)) {
					wrist.setGoalAngle(Constants.Wrist.kSafeAngleDegrees);
					setState(TowerState.GOING_TO_SAFE);
				} 
				break;
			}
			
			case GOING_TO_SAFE: {
				if (wrist.inPosition()){
					wrist.setIntakeSpeed(Constants.Wrist.kCoralHoldPower);
					joystick.setRumble(RumbleType.kBothRumble,0);
					
					setState(TowerState.GOT_CORAL);
				} else if (wrist.gotExitCoral()) {
						wrist.setIntakeSpeed(Constants.Wrist.kCoralHoldPower);
				}
				break;
			}

			case GOT_CORAL: {
				if (isTriggered(TowerEvent.GOTO_L1)){
					elevator.setGoalPositionMeters(Constants.Elevator.kL1CoralHeightMeters);
					currentLevel = 1;
					wrist.setIntakeSpeed(0);
					setState(TowerState.RAISING_TO_L123);
				} else if (isTriggered(TowerEvent.GOTO_L2)){
					elevator.setGoalPositionMeters(Constants.Elevator.kL2CoralHeightMeters);
					currentLevel = 2;
					wrist.setIntakeSpeed(0);
					setState(TowerState.RAISING_TO_L123);
				} else if (isTriggered(TowerEvent.GOTO_L3)) {
					elevator.setGoalPositionMeters(Constants.Elevator.kL3CoralHeightMeters);
					currentLevel = 3;
					wrist.setIntakeSpeed(0);
					setState(TowerState.RAISING_TO_L123);
				} else if (isTriggered(TowerEvent.GOTO_L4)){
					elevator.setGoalPositionMeters(Constants.Elevator.kL4CoralHeightMeters);
					currentLevel = 4;
					wrist.setIntakeSpeed(0);
					setState(TowerState.RAISING_TO_L4);
				} else if (isTriggered(TowerEvent.SCORE)){
					wrist.setIntakeSpeed(Constants.Wrist.kCoralIntakePower);
					setState(TowerState.FINISHING_SCORING_CORAL);
				} else {
					if (wrist.gotExitCoral()) {
						wrist.setIntakeSpeed(Constants.Wrist.kCoralHoldPower);
					} else {
						wrist.setIntakeSpeed(0);
					}
				}
				break;
			}
		
			case RAISING_TO_L123: {
				if (elevator.inPosition()) {
					setState(TowerState.READY_TO_SCORE_CORAL);
				}
				break;
			}

			case RAISING_TO_L4: {
				if (elevator.inPosition()) {
					wrist.setGoalAngle(Constants.Wrist.kL4AngleDegrees);
					setState(TowerState.TILTING_TO_SCORE_L4);
				}
				break;
			}

			case TILTING_TO_SCORE_L4: {
				if (wrist.inPosition()) {
					setState(TowerState.READY_TO_SCORE_CORAL);
				}
				break;
			}

			case READY_TO_SCORE_CORAL: {
				if (isTriggered(TowerEvent.SCORE)) {
					if (currentLevel == 4) {
						wrist.setIntakeSpeed(Constants.Wrist.kCoralL4ScoringPower);
					} else if (currentLevel > 1){
						wrist.setIntakeSpeed(Constants.Wrist.kCoralL23ScoringPower);
					} else {
						wrist.setIntakeSpeed(Constants.Wrist.kCoralL1ScoringPower);
					}
					
					setState(TowerState.SCORING_CORAL);
				} else if ((isTriggered(TowerEvent.GOTO_L4))  && (currentLevel != 4)){
					elevator.setGoalPositionMeters(Constants.Elevator.kL4CoralHeightMeters);
					currentLevel = 4;
					setState(TowerState.RAISING_TO_L4);
				} else if ((isTriggered(TowerEvent.GOTO_L3))  && (currentLevel != 3)){
					elevator.setGoalPositionMeters(Constants.Elevator.kL3CoralHeightMeters);
					wrist.setGoalAngle(Constants.Wrist.kSafeAngleDegrees);
					currentLevel = 3;
					setState(TowerState.RAISING_TO_L123);
				} else if ((isTriggered(TowerEvent.GOTO_L2))  && (currentLevel != 2)){
					elevator.setGoalPositionMeters(Constants.Elevator.kL2CoralHeightMeters);
					wrist.setGoalAngle(Constants.Wrist.kSafeAngleDegrees);
					currentLevel = 2;
					setState(TowerState.RAISING_TO_L123);
				} else if ((isTriggered(TowerEvent.GOTO_L1))  && (currentLevel != 1)){
					elevator.setGoalPositionMeters(Constants.Elevator.kL1CoralHeightMeters);
					wrist.setGoalAngle(Constants.Wrist.kSafeAngleDegrees);
					currentLevel = 1;
					setState(TowerState.RAISING_TO_L123);
				} else {
					if (wrist.gotExitCoral()) {
						wrist.setIntakeSpeed(Constants.Wrist.kCoralHoldPower);
					} else {
						wrist.setIntakeSpeed(0);
					}
				}
				break;
			}

			case SCORING_CORAL: {
				if ((!wrist.gotExitCoral() && stateTimer.hasElapsed(0.2)) ||
					(stateTimer.hasElapsed(0.5))) {
					wrist.setGoalAngle(Constants.Wrist.kSafeAngleDegrees);
					setState(TowerState.FINISHING_SCORING_CORAL);
				}
				break;
			}

			case FINISHING_SCORING_CORAL: {
				if (wrist.inPosition()) {
					if (goDirectAlgaeLevel > 0){  // special bypass to go to pickup Algae in Auto
						if (goDirectAlgaeLevel == 2){
							elevator.setGoalPositionMeters(Constants.Elevator.kL2AlgaeHeightMeters);
						} else if (goDirectAlgaeLevel == 3){
							elevator.setGoalPositionMeters(Constants.Elevator.kL3AlgaeHeightMeters);
						}
						wrist.setGoalAngle(Constants.Wrist.kAlgaeIntakeAngleDegrees);
						goDirectAlgaeLevel = 0;
						setState(TowerState.GOING_TO_ALGAE_INTAKE);
					} else {
						elevator.setGoalPositionMeters(Constants.Elevator.kIntakeHeightMeters);
						setState(TowerState.LOWERING);
					}
				}
				break;
			}

			case LOWERING: {
				if (elevator.inPosition()){
					wrist.setIntakeSpeed(0);
					wrist.setGoalAngle(Constants.Wrist.kIntakeAngleDegrees);
					setState(TowerState.GOING_TO_CORAL_INTAKE_ANGLE);
				}
				break;
			}
			
			case GOING_TO_CORAL_INTAKE_ANGLE: {
				if (wrist.inPosition()){
					setState(TowerState.HOME);
				}
				break;
			}

			// ====  ALGAE specific states  ========================================

			case GET_SAFE_FOR_L2_ALGAE: {
				if(wrist.inPosition()){
					elevator.setGoalPositionMeters(Constants.Elevator.kL2AlgaeHeightMeters);
					wrist.setGoalAngle(Constants.Wrist.kAlgaeIntakeAngleDegrees);
					setState(TowerState.GOING_TO_ALGAE_INTAKE);
				}
				break;
			}

			case GET_SAFE_FOR_L3_ALGAE: {
				if(wrist.inPosition()){
					elevator.setGoalPositionMeters(Constants.Elevator.kL3AlgaeHeightMeters);
					wrist.setGoalAngle(Constants.Wrist.kAlgaeIntakeAngleDegrees);
					setState(TowerState.GOING_TO_ALGAE_INTAKE);
				}
				break;
			}

			case GOING_TO_ALGAE_INTAKE: {
				if(elevator.inPosition() && wrist.inPosition()){
					wrist.setIntakeSpeed(Constants.Wrist.kAlgaeIntakePower);
					setState(TowerState.WAITING_FOR_ALGAE);
				}
				break;
			}

			case WAITING_FOR_ALGAE: {
				if(isTriggered(TowerEvent.SCORE)){
					if (currentLevel < 4) {
						wrist.setIntakeSpeed(Constants.Wrist.kAlgaeScoringPower);  // Score Algae
						Globals.GOT_ALGAE = false;
						setState(TowerState.FINISHING_L123_ALGAE);
					} else {
						// Lobbing into Barge
						wrist.setGoalAngle(Constants.Wrist.kAlgaeReleaseGoalAngleDegrees);
						setState(TowerState.WINDING_UP);
					}
				} else if (isTriggered(TowerEvent.GOTO_L1)) {
					currentLevel = 1;
					elevator.setGoalPositionMeters(Constants.Elevator.kL1AlgaeHeightMeters);
					wrist.setGoalAngle(Constants.Wrist.kAlgaeIntakeAngleDegrees);
					Globals.IDENTIFIED_TARGET = ApproachTarget.PROCESSOR;
					setState(TowerState.CHANGING_ALGAE_HEIGHT);
				} else if (isTriggered(TowerEvent.GOTO_L2)) {
					currentLevel = 2;
					elevator.setGoalPositionMeters(Constants.Elevator.kL2AlgaeHeightMeters);
					wrist.setGoalAngle(Constants.Wrist.kAlgaeIntakeAngleDegrees);
					setState(TowerState.CHANGING_ALGAE_HEIGHT);
				} else if (isTriggered(TowerEvent.GOTO_L3)) {
					currentLevel = 3;
					elevator.setGoalPositionMeters(Constants.Elevator.kL3AlgaeHeightMeters);
					wrist.setGoalAngle(Constants.Wrist.kAlgaeIntakeAngleDegrees);
					setState(TowerState.CHANGING_ALGAE_HEIGHT);
				} else if (isTriggered(TowerEvent.GOTO_L4)) {
					currentLevel = 4;
					elevator.setGoalPositionMeters(Constants.Elevator.kL4AlgaeWindupHeightMeters);
					wrist.setGoalAngle(Constants.Wrist.kAlgaeWindupAngleDegrees);
					setState(TowerState.CHANGING_ALGAE_HEIGHT);
				} else if(wrist.getIntakeCurrent() > Constants.Wrist.kAlgaeGrabbedCurrent){
					Globals.GOT_ALGAE = true;
				}
				break;
			}

			case CHANGING_ALGAE_HEIGHT: {
				if (elevator.inPosition()) {
					setState(TowerState.WAITING_FOR_ALGAE);
				}
				break;
			}

			case WINDING_UP: {
				if (wrist.getWristAngle() < Constants.Wrist.kAlgaeReleaseAngleDegrees){
					wrist.setIntakeSpeed(Constants.Wrist.kAlgaeScoringPower);
					Globals.GOT_ALGAE = false;
					setState(TowerState.ALGAE_BACKSPIN);
				}
				break;
			}

			case ALGAE_BACKSPIN: {
				if (wrist.getWristAngle() <= Constants.Wrist.kAlgaeBackspinDegrees){
					wrist.setIntakeSpeed(-Constants.Wrist.kAlgaeScoringPower);

					setState(TowerState.FINISHING_L4_ALGAE);
				}
				break;
			}

			case FINISHING_L4_ALGAE: {
				if (wrist.inPosition()){
					setState(TowerState.DONE_SCORING_ALGAE);
				}
				break;
			}

			case FINISHING_L123_ALGAE: {
				if (stateTimer.hasElapsed(0.5)){
					setState(TowerState.DONE_SCORING_ALGAE);
				}
				break;
			}

			case DONE_SCORING_ALGAE: {  
				if(goDirectAlgaeLevel > 0){  // special bypass to go to pickup Algae in Auto
					if (goDirectAlgaeLevel == 2){
						elevator.setGoalPositionMeters(Constants.Elevator.kL2AlgaeHeightMeters);
						currentLevel = 2;
					} else if (goDirectAlgaeLevel == 3){
						elevator.setGoalPositionMeters(Constants.Elevator.kL3AlgaeHeightMeters);
						currentLevel = 3;
					}

					wrist.setGoalAngle(Constants.Wrist.kAlgaeIntakeAngleDegrees);
					wrist.setIntakeSpeed(Constants.Wrist.kAlgaeIntakePower);
					goDirectAlgaeLevel = 0;
					setState(TowerState.CHANGING_ALGAE_HEIGHT);
				} else {
					wrist.setGoalAngle(Constants.Wrist.kSafeAngleDegrees);
					//elevator.setGoalPositionMeters(Constants.Elevator.kIntakeHeightMeters);
					currentLevel = 0;
					setState(TowerState.FINISHING_SCORING_CORAL);  // make sure the wrist is back before lowring
				}
				break;
			}

		}
	}

	@AutoLogOutput
	public double getTowerSpeedSafetyFactor() {
		//  Determine what portion of full speed can be used based on the tower State
		safetyFactor = 1;

		if ((currentState == TowerState.SCORING_CORAL) || (currentState == TowerState.FINISHING_SCORING_CORAL)) {
			safetyFactor = 0.25;
		} else if (elevator.getHeightMeters() > Constants.Elevator.kElevatorSpeedSafeHeightMeters) {
			double currentSafeHeight = elevator.getHeightMeters() - Constants.Elevator.kElevatorSpeedSafeHeightMeters;
			double maxSafeHeight = Constants.Elevator.kElevatorMaxHeightMeters - Constants.Elevator.kElevatorSpeedSafeHeightMeters;
			double ratio =  currentSafeHeight / maxSafeHeight;
			safetyFactor = 1.0 - (0.5 * ratio) ;
		}

		return safetyFactor;
	}

	public void triggerEvent(TowerEvent event){
		pendingEvent = event;
	}

	public TowerEvent getPendingEvent() {
		return pendingEvent;
	}
	
	public TowerState getState() {
		return currentState;
	}

	// -- Private Methods  ----------------------------------------------------
	private void updateDashboard() {
		SmartDashboard.putString("Tower State", currentState.toString() + " <- " + pendingEvent.toString());
		SmartDashboard.putNumber("Safety Factor", safetyFactor * 100);
		SmartDashboard.putNumber("Elevator Level", currentLevel);
	}
	
	private Boolean isTriggered(TowerEvent event){
		if (pendingEvent == event) {
			pendingEvent = TowerEvent.NONE;
			return true;
		} else {
			return false;
		}
	}

	private Boolean isHoldingGoTo(){
		if ((pendingEvent == TowerEvent.GOTO_L1) || (pendingEvent == TowerEvent.GOTO_L2) || 
		    (pendingEvent == TowerEvent.GOTO_L3) || (pendingEvent == TowerEvent.GOTO_L4)) {
			return true;
		} else {
			return false;
		}
	}

	private void setState(TowerState newState){
		currentState = newState;
		stateTimer.reset();
	}
}
