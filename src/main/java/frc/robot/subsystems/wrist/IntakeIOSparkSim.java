// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

public class IntakeIOSparkSim implements IntakeIO {
    private final DCMotor gearbox = DCMotor.getNeoVortex(1);

    private final DCMotorSim sim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                gearbox, 
                0.004, 
                1.0), 
            gearbox);

    private final SparkFlex intakeSpark;

    private final SparkFlexSim intakeMotor;
    private final SparkRelativeEncoderSim intakeEncoder;
    
    public IntakeIOSparkSim(IntakeIOSparkFlex io) {
        intakeSpark = io.getIntakeSpark();

        intakeMotor = new SparkFlexSim(intakeSpark, gearbox);
        intakeEncoder = intakeMotor.getRelativeEncoderSim();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        sim.setInputVoltage(
            intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
        sim.update(Constants.kDt);

        intakeMotor.iterate(
            sim.getAngularVelocityRPM() / 60, // RPS
            RoboRioSim.getVInVoltage(), 
            Constants.kDt);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                sim.getCurrentDrawAmps()));

        inputs.intakeMotorConnected = true;
        inputs.intakeVelocityRotPerSec = intakeEncoder.getVelocity();
        inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.intakeCurrentAmps = intakeMotor.getMotorCurrent();
    }

    @Override
    public void setSpeed(double speed) {
        intakeMotor.setAppliedOutput(speed);
    }

    @Override
    public void resetControl() {
        setSpeed(0.0);
    }
}
