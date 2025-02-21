// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
	private SparkFlex wristMotor;
	private RelativeEncoder wristEncoder;

	/** Creates a new WristSubsystem. */
	public WristSubsystem() {
		wristMotor = new SparkFlex(Constants.Wrist.kWristMotorId, SparkLowLevel.MotorType.kBrushless);

		wristEncoder = wristMotor.getEncoder();
	}

	public void set(double speed) {
		wristMotor.set(speed);
	}

	public double getPosition() {
		return wristEncoder.getPosition();
	}

	public double getVelocity() {
		return wristEncoder.getVelocity();
	}

	public void setIntakeSpeed(double speed){
		//Run the intake motor at this speed
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
		// SmartDashboard.putNumber("Wrist Velocity", wristEncoder.getVelocity());
	}
}
