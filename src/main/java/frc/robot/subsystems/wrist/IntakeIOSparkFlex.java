// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants;

public class IntakeIOSparkFlex implements IntakeIO {
    private final SparkFlex intakeSpark;
    private final RelativeEncoder intakeEncoder;
    private final SparkFlexConfig intakeConfig = new SparkFlexConfig();

    private final SparkFlexConfig resetFrameRateConfig = new SparkFlexConfig();

    public IntakeIOSparkFlex() {
        intakeSpark = new SparkFlex(Constants.Wrist.kIntakeMotorId, MotorType.kBrushless);
        intakeEncoder = intakeSpark.getEncoder();

        intakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.Wrist.kIntakeCurrentLimit);
        intakeConfig.encoder
            .positionConversionFactor(1.0) // meters
            .velocityConversionFactor(1.0 / 60.0); // meters per second

        intakeSpark.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetFrameRateConfig.signals.appliedOutputPeriodMs(10);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorConnected = true;
        inputs.intakeVelocityRotPerSec = intakeEncoder.getVelocity();
        inputs.intakeAppliedVolts = intakeSpark.getAppliedOutput() * intakeSpark.getBusVoltage();
        inputs.intakeCurrentAmps = intakeSpark.getOutputCurrent();
    }

    @Override
    public void setSpeed(double speed) {

    }

    @Override
    public void resetControl() {
        setSpeed(0.0);
    }

    @Override
    public void resetFrameRate() {
        intakeSpark.configure(resetFrameRateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
