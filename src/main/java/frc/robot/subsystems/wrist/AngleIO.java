// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface AngleIO {
    @AutoLog
    public static class AngleIOInputs {
        public boolean angleMotorConnected = false;
        public double anglePositionRot = 0.0;
        public double angleVelocityRotPerSec = 0.0;
        public double angleAppliedVolts = 0.0;
        public double angleCurrentAmps = 0.0;

        public boolean angleEncoderConnected = false;
        public double angleEncoderPositionDeg = 0.0;
        public double angleEncoderVelocityDegPerSec = 0.0;

        public double goalPositionDegrees = 0.0;
    }

    public default void updateInputs(AngleIOInputs inputs) {}

    public default void setGoalAngleDegrees(double angle) {}

    public default void runClosedLoop() {}

    public default void resetControl() {}

    public default void resetFrameRate() {}
}
