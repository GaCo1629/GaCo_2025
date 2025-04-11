// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import java.util.zip.ZipEntry;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;

/** Add your docs here. */
public class AngleIOSparkFlex implements AngleIO {
    private final SparkFlex angleSpark;
    private final AbsoluteEncoder angleEncoder;
    private final SparkClosedLoopController angleController;
    private final SparkFlexConfig angleConfig = new SparkFlexConfig();
    private final TrapezoidProfile angleTrapezoidProfile;
    private TrapezoidProfile.State angleGoal = new TrapezoidProfile.State();
    private TrapezoidProfile.State angleSetpoint;

    private final SparkFlexConfig resetFrameRateConfig = new SparkFlexConfig();

    public AngleIOSparkFlex() {
        angleSpark = new SparkFlex(Constants.Wrist.kAngleMotorId, MotorType.kBrushless);
        angleEncoder = angleSpark.getAbsoluteEncoder();
        angleController = angleSpark.getClosedLoopController();

        angleConfig
            .idleMode(IdleMode.kBrake)
            //.inverted(true)
            .smartCurrentLimit(Constants.Wrist.kAngleCurrentLimit);
        angleConfig.absoluteEncoder
            //.inverted(true)
            .positionConversionFactor(Constants.Wrist.kAngleFactor) // degrees
            .velocityConversionFactor(Constants.Wrist.kAngleFactor / 60.0); // degrees per second
        angleConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(Constants.Wrist.kP, Constants.Wrist.kI, Constants.Wrist.kD)
            .outputRange(-Constants.Wrist.kAnglePower, Constants.Wrist.kAnglePower)
            .positionWrappingInputRange(0, Constants.Wrist.kAngleFactor)
            .positionWrappingEnabled(true);

        angleSpark.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        angleTrapezoidProfile = new TrapezoidProfile(new Constraints(
                                                        Constants.Wrist.kAngleMaxVelocityDPS,
                                                        Constants.Wrist.kAngleMaxAccelerationDPSPS));
        angleSetpoint = new TrapezoidProfile.State(angleEncoder.getPosition(), angleEncoder.getVelocity());

        resetFrameRateConfig.signals.appliedOutputPeriodMs(10);
    }

    @Override
    public void updateInputs(AngleIOInputs inputs) {
        inputs.angleMotorConnected = true;
        inputs.anglePositionRot = angleSpark.getEncoder().getPosition();
        inputs.angleVelocityRotPerSec = angleSpark.getEncoder().getVelocity();
        inputs.angleAppliedVolts = angleSpark.getAppliedOutput() * angleSpark.getBusVoltage();
        inputs.angleCurrentAmps = angleSpark.getOutputCurrent();

        inputs.angleEncoderConnected = true;
        inputs.angleEncoderPositionDeg = angleEncoder.getPosition();
        inputs.angleEncoderVelocityDegPerSec = angleEncoder.getVelocity();

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

    @Override
    public void resetFrameRate() {
        angleSpark.configure(resetFrameRateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    protected SparkFlex getAngleSpark() {
        return angleSpark;
    }
}