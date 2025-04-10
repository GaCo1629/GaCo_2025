// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;

public class ElevatorIOSparkFlex implements ElevatorIO {
    private final SparkFlex leftElevatorMotor;
    private final SparkFlex centerElevatorMotor; // Cannot be final due to sysID Routine method
    private final SparkFlex rightElevatorMotor;

    private final SparkFlexConfig leftElevatorMotorConfig = new SparkFlexConfig();
    private final SparkFlexConfig centerElevatorMotorConfig = new SparkFlexConfig();
    private final SparkFlexConfig rightElevatorMotorConfig = new SparkFlexConfig();
    private final SparkFlexConfig resetFrameRateConfig = new SparkFlexConfig();
    private final SparkClosedLoopController elevatorController;
    private final RelativeEncoder elevatorEncoder;

    private final ElevatorFeedforward elevatorFeedforward;

	private final TrapezoidProfile elevatorTrapezoidProfile;
	private TrapezoidProfile.State elevatorGoal = new TrapezoidProfile.State();
	private TrapezoidProfile.State elevatorSetpoint = new TrapezoidProfile.State();

    public ElevatorIOSparkFlex() {
        leftElevatorMotor = new SparkFlex(Elevator.kElevatorMotorLeftId, MotorType.kBrushless);
        centerElevatorMotor = new SparkFlex(Elevator.kElevatorMotorCenterId, MotorType.kBrushless);
        rightElevatorMotor = new SparkFlex(Elevator.kElevatorMotorRightId, MotorType.kBrushless);

        elevatorController = centerElevatorMotor.getClosedLoopController();
        elevatorEncoder = centerElevatorMotor.getEncoder();
    
        elevatorFeedforward = new ElevatorFeedforward(Elevator.kS, Elevator.kG, Elevator.kV);

        elevatorTrapezoidProfile = new TrapezoidProfile(new Constraints(elevatorFeedforward.maxAchievableVelocity(12.0, Elevator.kElevatorMaxAccelerationMPSPS),
                                                                Elevator.kElevatorMaxAccelerationMPSPS));

        centerElevatorMotorConfig.closedLoop
            .p(Elevator.kP)
            .i(Elevator.kI)
            .d(Elevator.kD)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        centerElevatorMotorConfig.encoder
            .positionConversionFactor(Elevator.kElevatorEncoderPositionConversionFactor)
            .velocityConversionFactor(Elevator.kElevatorEncoderVelocityConversionFactor);
        centerElevatorMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Elevator.kElevatorCurrentLimit);

        leftElevatorMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Elevator.kElevatorCurrentLimit)
            .follow(centerElevatorMotor);
        leftElevatorMotorConfig.encoder
            .positionConversionFactor(Elevator.kElevatorEncoderPositionConversionFactor)
            .velocityConversionFactor(Elevator.kElevatorEncoderVelocityConversionFactor);
        
        rightElevatorMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Elevator.kElevatorCurrentLimit)
            .follow(centerElevatorMotor);
        rightElevatorMotorConfig.encoder
            .positionConversionFactor(Elevator.kElevatorEncoderPositionConversionFactor)
            .velocityConversionFactor(Elevator.kElevatorEncoderVelocityConversionFactor);

        leftElevatorMotor.configure(leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        centerElevatorMotor.configure(centerElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightElevatorMotor.configure(rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorEncoder.setPosition(Constants.Elevator.elevatorHomeHeightMeters); 

        elevatorSetpoint.position = elevatorEncoder.getPosition();
        elevatorSetpoint.velocity = elevatorEncoder.getVelocity();

        resetFrameRateConfig.signals.appliedOutputPeriodMs(10);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.encoderPositionMeters = elevatorEncoder.getPosition();
        inputs.encoderVelocityMetersPerSec = elevatorEncoder.getVelocity();
        
        inputs.motor1AppliedVolts = leftElevatorMotor.getAppliedOutput() * leftElevatorMotor.getBusVoltage();
        inputs.motor1CurrentAmps = leftElevatorMotor.getOutputCurrent();
        inputs.motor1PositionMeters = leftElevatorMotor.getEncoder().getPosition();
        inputs.motor1VelocityMetersPerSec = leftElevatorMotor.getEncoder().getVelocity();

        inputs.motor2AppliedVolts = centerElevatorMotor.getAppliedOutput() * centerElevatorMotor.getBusVoltage();
        inputs.motor2CurrentAmps = centerElevatorMotor.getOutputCurrent();
        inputs.motor2PositionMeters = centerElevatorMotor.getEncoder().getPosition();
        inputs.motor2VelocityMetersPerSec = centerElevatorMotor.getEncoder().getVelocity();

        inputs.motor3AppliedVolts = rightElevatorMotor.getAppliedOutput() * rightElevatorMotor.getBusVoltage();
        inputs.motor3CurrentAmps = rightElevatorMotor.getOutputCurrent();
        inputs.motor3PositionMeters = rightElevatorMotor.getEncoder().getPosition();
        inputs.motor3VelocityMetersPerSec = rightElevatorMotor.getEncoder().getVelocity();

        
        inputs.goalPositionMeters = elevatorGoal.position;
    }

    @Override
    public void resetEncoder() {
        elevatorEncoder.setPosition(Constants.Elevator.elevatorHomeHeightMeters); 
    }

    @Override
    public void resetFrameRate() {
        leftElevatorMotor.configure(resetFrameRateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        centerElevatorMotor.configure(resetFrameRateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rightElevatorMotor.configure(resetFrameRateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setGoalPositionMeters(double goalPositionMeters) {
        if (goalPositionMeters < Constants.Elevator.kElevatorMinHeightMeters) {
            goalPositionMeters = Constants.Elevator.kElevatorMinHeightMeters;
        } else if (goalPositionMeters > Constants.Elevator.kElevatorMaxHeightMeters) {
            goalPositionMeters = Constants.Elevator.kElevatorMaxHeightMeters;
        }
          
        elevatorGoal.position = goalPositionMeters;
        elevatorGoal.velocity = 0.0;
      
        elevatorSetpoint.position = elevatorEncoder.getPosition();
        elevatorSetpoint.velocity = 0.0;
    } 

    @Override
    public void runClosedLoop() {
        elevatorSetpoint = elevatorTrapezoidProfile.calculate(Constants.kDt, elevatorSetpoint, elevatorGoal);
        double arbFF = elevatorFeedforward.calculate(elevatorSetpoint.velocity);
        elevatorController.setReference(elevatorSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFF);
    }

    @Override
    public void setSpeed(double speed) {
        centerElevatorMotor.set(speed);
    }
}
