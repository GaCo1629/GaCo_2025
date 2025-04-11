// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DefaultWristCmd;
import frc.robot.subsystems.Globals;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;

public class Wrist extends SubsystemBase {
  private final AngleIO angleIO;
  private final IntakeIO intakeIO;
  private final SensorIO sensorIO;

  private final AngleIOInputsAutoLogged angleInputs = new AngleIOInputsAutoLogged();
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
  private final SensorIOInputsAutoLogged sensorInputs = new SensorIOInputsAutoLogged();
  
  /** Creates a new WristSubsystem. */
  public Wrist(AngleIO angleIO, IntakeIO intakeIO, SensorIO sensorIO) {
    this.angleIO = angleIO;
    this.intakeIO = intakeIO;
    this.sensorIO = sensorIO;

    setDefaultCommand(new DefaultWristCmd(this));
  }

  // The configuration interfaces may be accessed by typing in the IP address of the roboRIO into a web
  //  browser followed by :5812.
  @Override
  public void simulationPeriodic() {
    SmartDashboard.putNumber("Wrist Goal", angleInputs.goalPositionDegrees);    
    SmartDashboard.putNumber("Wrist Angle", angleInputs.angleEncoderPositionDeg);
    SmartDashboard.putString("Wrist Power", "SIMULATION");
    SmartDashboard.putNumber("Exit Coral Sensor", sensorInputs.exitTOFRangeMM);
    SmartDashboard.putNumber("Enter Coral Sensor", sensorInputs.exitTOFRangeMM);
  }

  @Override
  public void periodic() {
    angleIO.updateInputs(angleInputs);
    Logger.processInputs("Wrist/Angle", angleInputs);

    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Wrist/Intake", intakeInputs);

    sensorIO.updateInputs(sensorInputs);
    Logger.processInputs("Wrist/Sensors", sensorInputs);

    Globals.GOT_CORAL = gotExitCoral() && gotEnterCoral();

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Goal", angleInputs.goalPositionDegrees);    
    SmartDashboard.putNumber("Wrist Angle", angleInputs.angleEncoderPositionDeg);
    SmartDashboard.putNumber("Wrist Power", angleInputs.angleAppliedVolts);
    SmartDashboard.putNumber("Exit Coral Sensor", sensorInputs.exitTOFRangeMM);
    SmartDashboard.putNumber("Enter Coral Sensor", sensorInputs.exitTOFRangeMM);
    
  }

  public void initialize() {
    intakeIO.setSpeed(0);
  }

  public void resetWristControl() {
    angleIO.resetControl();
    intakeIO.resetControl();
  }

  public void resetFrameRate() {
    angleIO.resetFrameRate();
    intakeIO.resetFrameRate();
  }

  public void setGoalAngle(double angle) {
    angleIO.setGoalAngleDegrees(angle);
  }

  public void setIntakeSpeed(double speed) {
    intakeIO.setSpeed(speed);
  }

  public void runWristClosedLoop() {
    angleIO.runClosedLoop();
  }

  public boolean gotExitCoral() {
    return sensorInputs.exitCoral;
  }

  public boolean gotEnterCoral() {
    return sensorInputs.enterCoral;
  }

  public double getIntakeSpeed() {
    return intakeInputs.intakeVelocityRotPerSec;
  }

  public double getIntakeCurrent() {
    return intakeInputs.intakeCurrentAmps;
  }

  public double getWristAngle(){
    return angleInputs.angleEncoderPositionDeg;
  }

  public double getWristSpeed(){
    return angleInputs.angleEncoderVelocityDegPerSec;
  }

  public boolean inPosition(){
    if (Utils.isSimulation()){
      Globals.WRIST_IN_POSITION = true;
      return Globals.WRIST_IN_POSITION;
    } else {
      Globals.WRIST_IN_POSITION = (Math.abs(angleInputs.goalPositionDegrees - angleInputs.angleEncoderPositionDeg) < Constants.Wrist.kAngleTollerance);
      return Globals.WRIST_IN_POSITION;
    }
  }
}
