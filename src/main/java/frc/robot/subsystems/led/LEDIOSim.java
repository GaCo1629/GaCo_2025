// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.Globals;

public class LEDIOSim implements LEDIO {
    private final AddressableLEDSim ledStrip;

    private AddressableLEDBuffer ledBuffer;  // Use the new class that flips the R&G LEDs

    private byte[] ledByteBuffer;

    private final Timer ledTimer = new Timer();
    private LEDmode lastMode = LEDmode.NONE;
    private int patternMarker = 0;
    private int direction = 0;
    private boolean stripOn = false;

    // The LED bar is divided into 3 bands...
    // 0-4   ELV In Pos    
    // 5-8   Wrist in Pos
    // 9-16  Got Coral
    // 17-20 Wrist in Pos
    // 21-44 ELV In Pos    
    private final AddressableLEDBufferView elevatorView_1 = ledBuffer.createView(0, 4);
    private final AddressableLEDBufferView elevatorView_2 = ledBuffer.createView(21, 25);
    private final AddressableLEDBufferView wristView_1 = ledBuffer.createView(5, 8);
    private final AddressableLEDBufferView wristView_2 = ledBuffer.createView(17, 20);
    private final AddressableLEDBufferView coralView = ledBuffer.createView(9, 16);

    public LEDIOSim(int port, int stripLength) {
        ledStrip = AddressableLEDSim.createForChannel(port);
        ledStrip.setLength(stripLength);

        ledBuffer = new AddressableLEDBuffer(stripLength);
        ledByteBuffer = new byte[stripLength];
        ledStrip.setData(ledByteBuffer);
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
        ledBuffer.forEach((int index, int r, int g, int b) -> {
            ledByteBuffer[index * 4] = (byte) b;
            ledByteBuffer[(index * 4) + 1] = (byte) g;
            ledByteBuffer[(index * 4) + 2] = (byte) r;
            ledByteBuffer[(index * 4) + 3] = 0;
        });
        ledStrip.setData(ledByteBuffer);
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
