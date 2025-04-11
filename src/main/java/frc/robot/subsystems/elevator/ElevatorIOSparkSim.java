// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;

public class ElevatorIOSparkSim implements ElevatorIO {
    private final DCMotor gearbox =
        DCMotor.getNeoVortex(3);

    private final ElevatorSim sim =
        new ElevatorSim(
            Elevator.kV / Elevator.kRelativeEncoderScaleRevToMeters,
            Elevator.kA,
            gearbox, 
            Constants.Elevator.kElevatorMinHeightMeters, 
            Constants.Elevator.kElevatorMaxHeightMeters + Constants.Elevator.kElevatorMinHeightMeters, 
            true, 
            Constants.Elevator.elevatorHomeHeightMeters);

    private final SparkFlex[] elevatorMotors;

    private final SparkFlexSim centerElevatorMotor;

    private final SparkClosedLoopController elevatorController;

    private final SparkRelativeEncoderSim elevatorEncoder;

    private final ElevatorFeedforward elevatorFeedforward;

	private final TrapezoidProfile elevatorTrapezoidProfile;
	private TrapezoidProfile.State elevatorGoal = new TrapezoidProfile.State();
	private TrapezoidProfile.State elevatorSetpoint = new TrapezoidProfile.State();

    public ElevatorIOSparkSim(ElevatorIOSparkFlex io) {
        elevatorMotors = io.getMotors();

        centerElevatorMotor = new SparkFlexSim(
            elevatorMotors[1], 
            gearbox);
        elevatorController = elevatorMotors[1].getClosedLoopController();
        elevatorEncoder = centerElevatorMotor.getRelativeEncoderSim();
        
        elevatorFeedforward = new ElevatorFeedforward(
            Elevator.kS, 
            Elevator.kG, 
            Elevator.kV);

        elevatorTrapezoidProfile = new TrapezoidProfile(
            new Constraints(
                elevatorFeedforward.maxAchievableVelocity(
                    12.0, 
                    Elevator.kElevatorMaxAccelerationMPSPS),
                Elevator.kElevatorMaxAccelerationMPSPS));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sim.setInput(
            centerElevatorMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
        sim.update(Constants.kDt);

        centerElevatorMotor.iterate(
            sim.getVelocityMetersPerSecond(),
            RoboRioSim.getVInVoltage(),
            Constants.kDt);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                sim.getCurrentDrawAmps()));

        inputs.encoderConnected = true;
        inputs.encoderPositionMeters = elevatorEncoder.getPosition();
        inputs.encoderVelocityMetersPerSec = elevatorEncoder.getVelocity();

        inputs.motor2Connected = true;
        inputs.motor2PositionMeters = centerElevatorMotor.getPosition();
        inputs.motor2VelocityMetersPerSec = centerElevatorMotor.getVelocity();
        inputs.motor2AppliedVolts = 
            centerElevatorMotor.getAppliedOutput() * RoboRioSim.getVInVoltage();
        inputs.motor2CurrentAmps = centerElevatorMotor.getMotorCurrent();

        inputs.goalPositionMeters = elevatorGoal.position;
        inputs.totalCurrent = sim.getCurrentDrawAmps();
    }

    @Override
    public void resetEncoder() {
        elevatorEncoder.setPosition(Elevator.elevatorHomeHeightMeters); 
    }

    @Override
    public void setGoalPositionMeters(double goalPositionMeters) {
        if (goalPositionMeters < Elevator.kElevatorMinHeightMeters) {
            goalPositionMeters = Elevator.kElevatorMinHeightMeters;
        } else if (goalPositionMeters > Elevator.kElevatorMaxHeightMeters) {
            goalPositionMeters = Elevator.kElevatorMaxHeightMeters;
        }
          
        elevatorGoal.position = goalPositionMeters;
        elevatorGoal.velocity = 0.0;
      
        elevatorSetpoint.position = elevatorEncoder.getPosition();
        elevatorSetpoint.velocity = 0.0;
    } 

    @Override
    public void runClosedLoop() {
        elevatorSetpoint = 
            elevatorTrapezoidProfile.calculate(
                Constants.kDt, 
                elevatorSetpoint, 
                elevatorGoal);

        double arbFF = 
            elevatorFeedforward.calculate(
                elevatorSetpoint.velocity);
                
        elevatorController.setReference(
            elevatorSetpoint.position, 
            ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, 
            arbFF);
    }

    @Override
    public void setSpeed(double speed) {
        centerElevatorMotor.setAppliedOutput(speed);
    }
}
