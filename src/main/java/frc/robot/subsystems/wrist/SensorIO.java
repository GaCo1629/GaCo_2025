// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

import com.playingwithfusion.TimeOfFlight.Status;

public interface SensorIO {
    @AutoLog
    public static class SensorIOInputs {
        public Status enterTOFStatus = Status.Invalid;
        public double enterTOFRangeMM = 0.0;
        public boolean enterCoral = false;

        public Status exitTOFStatus = Status.Invalid;
        public double exitTOFRangeMM = 0.0;
        public boolean exitCoral = false;
    }

    public default void updateInputs(SensorIOInputs inputs) {}
}
