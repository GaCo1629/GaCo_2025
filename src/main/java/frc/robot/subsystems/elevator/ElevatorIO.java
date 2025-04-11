// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean motor1Connected = false;
        public double motor1PositionMeters = 0.0;
        public double motor1VelocityMetersPerSec = 0.0;
        public double motor1AppliedVolts = 0.0;
        public double motor1CurrentAmps = 0.0;

        public boolean motor2Connected = false;
        public double motor2PositionMeters = 0.0;
        public double motor2VelocityMetersPerSec = 0.0;
        public double motor2AppliedVolts = 0.0;
        public double motor2CurrentAmps = 0.0;

        public boolean motor3Connected = false;
        public double motor3PositionMeters = 0.0;
        public double motor3VelocityMetersPerSec = 0.0;
        public double motor3AppliedVolts = 0.0;
        public double motor3CurrentAmps = 0.0;

        public boolean encoderConnected = false;
        public double encoderPositionMeters = 0.0;
        public double encoderVelocityMetersPerSec = 0.0;

        public double goalPositionMeters = 0.0;
        public double totalCurrent = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void resetEncoder() {}

    public default void resetFrameRate() {}

    public default void setGoalPositionMeters(double goalPositionMeters) {}

    public default void runClosedLoop() {}

    public default void setSpeed(double speed) {}
}
