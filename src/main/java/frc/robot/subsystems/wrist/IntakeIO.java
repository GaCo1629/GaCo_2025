// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean intakeMotorConnected = false;
        public double intakeVelocityRotPerSec = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double intakeCurrentAmps = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setSpeed(double speed) {}

    public default void resetControl() {}

    public default void resetFrameRate() {}
}
