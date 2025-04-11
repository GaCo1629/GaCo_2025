// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import frc.robot.Constants;

public class SensorIOPWF implements SensorIO {
    private final TimeOfFlight enterTOF;
    private final TimeOfFlight exitTOF;

    public SensorIOPWF() {
        exitTOF = new TimeOfFlight(Constants.Wrist.kExitTOFId);
        exitTOF.setRangingMode(RangingMode.Short, Constants.Wrist.kTOFSampleTime);
        exitTOF.setRangeOfInterest(0, 0, 15, 15);
        
        enterTOF = new TimeOfFlight(Constants.Wrist.kEnterTOFId);
        enterTOF.setRangingMode(RangingMode.Short, Constants.Wrist.kTOFSampleTime);
        enterTOF.setRangeOfInterest(0, 0, 15, 15);
    }

    @Override
    public void updateInputs(SensorIOInputs inputs) {
        inputs.enterTOFStatus = enterTOF.getStatus();
        inputs.enterTOFRangeMM = enterTOF.getRange();
        inputs.enterCoral = enterTOF.getRange() < Constants.Wrist.kMaxCoralDetectRangeMM;

        inputs.exitTOFStatus = exitTOF.getStatus();
        inputs.exitTOFRangeMM = exitTOF.getRange();
        inputs.exitCoral = exitTOF.getRange() < Constants.Wrist.kMaxCoralDetectRangeMM;
    }
}
