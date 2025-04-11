// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.Constants.Wrist;;

/** Add your docs here. */
public class AngleIOSparkSim implements AngleIO {
    private final DCMotor gearbox = DCMotor.getNeoVortex(1);

    private final DCMotorSim sim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                gearbox, 
                0.004, 
                Wrist.kAngleGearing), // 4:1 Gearing
            gearbox);
    
    private final SparkFlex angleSpark;

    private final SparkFlexSim angleMotor;
    private final SparkAbsoluteEncoderSim angleEncoder;
    private final SparkClosedLoopController angleController;
    private final TrapezoidProfile angleTrapezoidProfile;
    private TrapezoidProfile.State angleGoal = new TrapezoidProfile.State();
    private TrapezoidProfile.State angleSetpoint = new TrapezoidProfile.State();

    public AngleIOSparkSim(AngleIOSparkFlex io) {
        angleSpark = io.getAngleSpark();

        angleMotor = new SparkFlexSim(angleSpark, gearbox);
        angleEncoder = angleMotor.getAbsoluteEncoderSim();
        angleController = angleSpark.getClosedLoopController();

        angleTrapezoidProfile = new TrapezoidProfile(new Constraints(
                                                        Wrist.kAngleMaxVelocityDPS,
                                                        Wrist.kAngleMaxAccelerationDPSPS));
        angleSetpoint = new TrapezoidProfile.State(angleEncoder.getPosition(), angleEncoder.getVelocity());
    }

    @Override
    public void updateInputs(AngleIOInputs inputs) {
        sim.setInputVoltage(
            angleMotor.getAppliedOutput() * angleMotor.getBusVoltage());
        sim.update(Constants.kDt);

        angleMotor.iterate(
            sim.getAngularVelocityRPM(), // RPM
            RoboRioSim.getVInVoltage(), 
            Constants.kDt);

        angleEncoder.iterate(
            sim.getAngularVelocityRPM() / 60.0 * Wrist.kAngleFactor, // Degrees per second
            Constants.kDt);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                sim.getCurrentDrawAmps()));

        inputs.angleEncoderConnected = true;
        inputs.angleEncoderPositionDeg = angleEncoder.getPosition();
        inputs.angleEncoderVelocityDegPerSec = angleEncoder.getVelocity();

        inputs.angleMotorConnected = true;
        inputs.anglePositionRot = angleMotor.getPosition();
        inputs.angleVelocityRotPerSec = angleMotor.getVelocity() / 60;
        inputs.angleAppliedVolts = 
            angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
        inputs.angleCurrentAmps = angleMotor.getMotorCurrent();

        inputs.goalPositionDegrees = angleGoal.position;
    }

     @Override
    public void setGoalAngleDegrees(double angle) {
        angleGoal.position = angle;
        angleGoal.velocity = 0.0;
        
        angleSetpoint.position = angleEncoder.getPosition();
        angleSetpoint.velocity = 0.0;
    }

    @Override
    public void runClosedLoop() {
        angleSetpoint = angleTrapezoidProfile.calculate(Constants.kDt, angleSetpoint, angleGoal);
		angleController.setReference(angleSetpoint.position, ControlType.kPosition);
    }

    @Override
    public void resetControl() {
        setGoalAngleDegrees(angleEncoder.getPosition());
    }
}
