// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import frc.robot.subsystems.Globals;

public class LEDIOReal implements LEDIO {
    private final AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;  // Use the new class that flips the R&G LEDs

    private final Timer ledTimer = new Timer();
    private LEDmode lastMode = LEDmode.NONE;
    private int patternMarker = 0;
    private int direction = 0;
    private boolean stripOn = false;

    // The LED bar is divided into 3 bands...
    // 0-3   ELV In Pos    
    // 4-7   Wrist in Pos
    // 8-15  Got Coral
    // 16-19 Wrist in Pos
    // 20-23 ELV In Pos   
    private AddressableLEDBufferView elevatorView_1;
    private AddressableLEDBufferView elevatorView_2;
    private AddressableLEDBufferView wristView_1;
    private AddressableLEDBufferView wristView_2;
    private AddressableLEDBufferView coralView;

    public LEDIOReal(int port, int stripLength) {
        ledStrip = new AddressableLED(port);
        ledStrip.setColorOrder(ColorOrder.kGRB);
    
        ledBuffer = new AddressableLEDBuffer(stripLength);
        ledStrip.setLength(stripLength);

        // Set the data
        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    @Override
    public void createViews() {
        elevatorView_1 = ledBuffer.createView(0, 3);
        elevatorView_2 = ledBuffer.createView(20, 24);
        wristView_1 = ledBuffer.createView(4, 7);
        wristView_2 = ledBuffer.createView(16, 19);
        coralView = ledBuffer.createView(8, 15);
    }

    @Override
    public void updateInputs(LEDIOInputs inputs)
    {
        inputs.ledMode = Globals.getLEDMode();
        inputs.stripLength = ledBuffer.getLength();

        if (inputs.ledMode != lastMode) {
            clearStrip();
            ledTimer.restart();
            lastMode = inputs.ledMode;
        }
    }

    @Override
    public void setLEDMode(LEDmode mode) {
        Globals.setLEDMode(mode);
    }
    
    @Override
    public void setStrip(LEDPattern pattern) {
        pattern.applyTo(ledBuffer);
    }
    
    @Override
    public void clearStrip() {
        LEDPatterns.BLACK.applyTo(ledBuffer);
    }

    @Override
    public void showAlliance() {
        // turn off the last LED and then move to the next location.  Bounce at ends
        ledBuffer.setLED(patternMarker, Color.kBlack);

        if (patternMarker == 0) {
            direction = 1;
        } else if (patternMarker == (ledBuffer.getLength() - 1)) {
            direction = -1;
        }
        patternMarker += direction; // up or down

        // Set the LED color based on alliance color.  Green if unknown.
        if (DriverStation.getAlliance().isEmpty()) {
            ledBuffer.setLED(patternMarker, Color.kGreen);
        } else if (DriverStation.getAlliance().get() == Alliance.Red) {
            ledBuffer.setLED(patternMarker, Color.kRed);
        } else {
            ledBuffer.setLED(patternMarker, Color.kBlue);
        } 
    }

    @Override
    public void flashStrip(LEDPattern pattern, double onTime, double offTime) {
        if (stripOn && ledTimer.hasElapsed(onTime) ) {
            ledTimer.restart();
            if (offTime > 0) {
                clearStrip();
                stripOn = false;
            }
        } else if (!stripOn && ledTimer.hasElapsed(offTime)) {
            setStrip(pattern);
            ledTimer.restart();
            stripOn = true;
        }
    }

    @Override
    public void updateStrip() {
        ledStrip.setData(ledBuffer);
    }

    @Override
    public void showInPosition() {
        clearStrip();
  
        if (Globals.GOT_CORAL) {
            LEDPatterns.GREEN.applyTo(coralView);
        } else if (Globals.GOT_ALGAE) {
            LEDPatterns.YELLOW.applyTo(coralView);
        } else {
            LEDPatterns.RED.applyTo(coralView);
        }
  
        if (Globals.WRIST_IN_POSITION) {
            LEDPatterns.GREEN.applyTo(wristView_1);
            LEDPatterns.GREEN.applyTo(wristView_2);
        } else {
            LEDPatterns.RED.applyTo(wristView_1);
            LEDPatterns.RED.applyTo(wristView_2);
         }
        
        if (Globals.ELEVATOR_IN_POSITION) {
            LEDPatterns.GREEN.applyTo(elevatorView_1);
            LEDPatterns.GREEN.applyTo(elevatorView_2);
        } else {
            LEDPatterns.RED.applyTo(elevatorView_1);
            LEDPatterns.RED.applyTo(elevatorView_2);
        }
    }
}
