// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.LEDPattern;

public interface LEDIO {
    @AutoLog
    public static class LEDIOInputs {
        public LEDmode ledMode = LEDmode.NONE;
        public int port = 0;
        public int stripLength = 0;
    }

    public default void updateInputs(LEDIOInputs inputs) {}

    public default void setLEDMode(LEDmode mode) {}

    public default void setStrip(LEDPattern pattern) {}

    public default void clearStrip() {}

    public default void showAlliance() {}

    public default void flashStrip(LEDPattern pattern, double onTime, double offTime) {}

    public default void updateStrip() {}

    public default void showInPosition() {}
}
