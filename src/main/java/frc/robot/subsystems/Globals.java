package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.approach.ApproachTarget;
import frc.robot.subsystems.led.LEDmode;

public final class Globals extends SubsystemBase {

    @AutoLogOutput
    public  static boolean      GOT_CORAL;

    @AutoLogOutput
    public  static boolean      GOT_ALGAE;

    @AutoLogOutput
    public  static boolean      WRIST_IN_POSITION;

    @AutoLogOutput
    public  static boolean      ELEVATOR_IN_POSITION;

    @AutoLogOutput
    public  static ApproachTarget IDENTIFIED_TARGET;
    
    @AutoLogOutput  
    private static LEDmode      LED_MODE;

    public Globals(){
        GOT_CORAL = false;
        GOT_ALGAE = false;
        WRIST_IN_POSITION = false;
        ELEVATOR_IN_POSITION = false;
        IDENTIFIED_TARGET = ApproachTarget.UNKNOWN;    
        LED_MODE  = LEDmode.ALLIANCE;
    }

    @Override
	public void periodic() {
        SmartDashboard.putString("Approach Target", IDENTIFIED_TARGET.toString());
        SmartDashboard.putString("LED Mode", Globals.LED_MODE.toString());
    }

    public static void setLEDMode(LEDmode mode) {
        LED_MODE = mode;
    }

    public static LEDmode getLEDMode() {
        return LED_MODE;
    }
 }
