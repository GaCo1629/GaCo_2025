// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import frc.robot.subsystems.Globals;

public class LEDIOSim implements LEDIO {
    private LEDmode lastMode = LEDmode.NONE;

    @Override
    public void updateInputs(LEDIOInputs inputs)
    {
        inputs.ledMode = Globals.getLEDMode();

        if (inputs.ledMode != lastMode) {
            lastMode = inputs.ledMode;
        }
    }

    @Override
    public void setLEDMode(LEDmode mode) {
        Globals.setLEDMode(mode);
    }
}
