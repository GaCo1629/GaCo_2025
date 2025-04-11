// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.playingwithfusion.TimeOfFlight.Status;

/** Add your docs here. */
public class SensorIOSim implements SensorIO {
    @Override
    public void updateInputs(SensorIOInputs inputs) {
        inputs.enterTOFStatus = Status.Valid;
        inputs.enterTOFRangeMM = 5;
        inputs.enterCoral = true;

        inputs.exitTOFStatus = Status.Valid;
        inputs.exitTOFRangeMM = 5;
        inputs.exitCoral = false;
    }
}
