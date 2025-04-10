// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DefaultElevatorCmd;
import frc.robot.subsystems.Globals;

public class ElevatorSubsystem extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private double lastGoalPositionMeters = Constants.Elevator.kElevatorMinHeightMeters;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;

    setDefaultCommand(new DefaultElevatorCmd(this));
  }

  @Override
  public void simulationPeriodic() {
    SmartDashboard.putNumber("Elev Rel Hgt", Units.metersToInches(inputs.encoderPositionMeters));
    SmartDashboard.putNumber("ElevatorGoal", Units.metersToInches(inputs.goalPositionMeters));
    SmartDashboard.putString("Elevator Power", "SIMULATION");
  }

  @Override
	public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

		// This method will be called once per scheduler run
    SmartDashboard.putNumber("Elev Rel Hgt", Units.metersToInches(inputs.encoderPositionMeters));
		SmartDashboard.putNumber("Elevator Goal", Units.metersToInches(inputs.goalPositionMeters));
    SmartDashboard.putNumber("Elevator Power", inputs.motor2AppliedVolts);
    SmartDashboard.putNumber("Elevator Current", getCurrent());    
    SmartDashboard.putNumber("Elevator Velocity", inputs.encoderVelocityMetersPerSec);	
  }

  public void setGoalPositionMeters(double meters) {
    io.setGoalPositionMeters(inputs, meters);
  }

  public void runClosedLoop() {
    io.runClosedLoop();
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }
  
  public void bumpElevatorMeters(double changeMeters) {
    io.setGoalPositionMeters(inputs, lastGoalPositionMeters + changeMeters);
  }

  public void resetElevatorControl() {
    io.setGoalPositionMeters(inputs, inputs.encoderPositionMeters);
  }

  public void resetEncoder() {
    resetEncoder();
  }

  public double getHeightMeters(){
    return inputs.encoderPositionMeters;
  }

  public boolean inPosition(){
    if (Utils.isSimulation()){
      Globals.ELEVATOR_IN_POSITION = true;
      return Globals.ELEVATOR_IN_POSITION;
    } else {
      Globals.ELEVATOR_IN_POSITION = (Math.abs(inputs.goalPositionMeters - inputs.encoderPositionMeters) < Constants.Elevator.kHeightTolleranceMeters);
      return Globals.ELEVATOR_IN_POSITION;
    }
  }

  public double getCurrent() {
    return inputs.motor1CurrentAmps + inputs.motor2CurrentAmps + inputs.motor3CurrentAmps;
  }
}
