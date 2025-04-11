// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Globals;

public class LED extends SubsystemBase {
  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

  /** Creates a new LED Strip. */
  public LED(LEDIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LED" + Integer.toString(inputs.port), inputs);

    if (DriverStation.isDisabled()) {
      if (Globals.GOT_CORAL) {
        Globals.setLEDMode(LEDmode.ALLIANCE);
      } else {
        Globals.setLEDMode(LEDmode.ERROR);
      }
    } 

    switch (inputs.ledMode) {
      case NONE:
        io.clearStrip();
        break;
      case ALLIANCE:    // Display Alliance color
        io.showAlliance();
        break;
      case MANUAL:      // Show driving lights
        io.showInPosition();
        break;
      case APPROACH:    // Auto Approach
        io.flashStrip(LEDPatterns.BLUE, 0.25, 0.0);
        break;
      case ERROR:  // Displaying system error 
        default:
        io.flashStrip(LEDPatterns.PURPLE, 0.2, 0.2);
        break;
    }

    // Set the LEDs
    io.updateStrip();
  }
}

