// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

/** Add your docs here. */
public class Constants {

    public static final double kDt = 0.02;

    public class Wrist {
        public static final int kAngleMotorId = 61;
        public static final int kIntakeMotorId = 62;
        public static final int kExitTOFId = 63;
        public static final int kEnterTOFId = 64;
        
        public static final Angle kAngleFactor = Degrees.of(360 * 24 / 40); // 216 degrees

        public static final double kP = 0.04; // was 0.02
        public static final double kI = 0;
        public static final double kD = 0.0;
        public static final double kAngleTollerance = 1;
		
        public static final double kAnglePower = 0.5; // was 1.0
		
		public static final double kCoralIntakePower = -0.3;
        public static final double kCoralSlowIntakePower = -0.1;
        public static final double kCoralRetractPower = 0.02;
		public static final double kCoralOutputPower =  0.5;
		public static final double kCoralL234ScoringPower = -1.0;
        public static final double kCoralL1ScoringPower = -0.2;

		public static final double kAlgaeIntakePower = 0.3;
        public static final double kLowAlgaeIntakePower = 0.2;

        public static final AngularVelocity kAngleMaxVelocity = DegreesPerSecond.of(400); // 
		public static final AngularAcceleration kAngleMaxAcceleration = DegreesPerSecondPerSecond.of(1000); // 

        public static final Angle kIntakeAngle             = Degrees.of(4);
        public static final Angle kSafeAngle               = Degrees.of(30);
        public static final Angle kL4Angle                 = Degrees.of(48);
        public static final Angle kHighAlgaeAngle          = Degrees.of(60);

        public static final Angle kAlgaeIntakeAngle        = Degrees.of(180); 

        public static final Angle kAlgaeWindupAngle        = Degrees.of(150); 
        public static final Angle kAlgaeReleaseAngle       = Degrees.of(60); 
        
        public static final Angle kMaxAngleWhenHome        = Degrees.of(182);
        public static final Angle kMaxAngle                = Degrees.of(210);

        public static final Distance kMaxCoralDetectRangeMM   = Millimeters.of(80);
    }

    public class Elevator {

        public static final Distance elevatorHomeHeight = Inches.of(17.5);  // only valid when elevator is homed;

        // scale factors
        public static final double kRelativeEncoderScaleRevToMeters = 0.0315;  // 25 turns for 31 inches of travel
        public static final double kAbsoluteEncoderScaleVoltsToMeters = 0.498;
        public static final double kAbsoluteEncoderOffsetVoltsToMeters = 0.43;

        public static final double kP = 6;  // was 8
        public static final double kI = 0;
        public static final double kD = 0.46;

        public static final double kS = 0.04435;    //  these may need to be rescaled for meters by dividing 
		public static final double kG = 0.257;      //  by kRelativeEncoderScaleRevToMeters
		public static final double kV = 0.117 ;     //
		public static final double kA = 0.00803 ;   //

        public static final Distance kHeightTollerance = Inches.of(1.0);

        public static final Current kElevatorCurrentLimit = Amps.of(60);

        public static final int kElevatorMotorLeftId = 51;
        public static final int kElevatorMotorCenterId = 52;
        public static final int kElevatorMotorRightId = 53;

        public static final Distance kElevatorMaxHeight = Inches.of(73);
        public static final Distance kElevatorSpeedSafeHeight = Inches.of(40);
        public static final Distance kElevatorMinHeight = Inches.of(17.5);

        public static final Distance kIntakeHeight = Inches.of(17.5); 

        public static final Distance kL1CoralHeight = Inches.of(21);
        public static final Distance kL2CoralHeight = Inches.of(31);
        public static final Distance kL3CoralHeight = Inches.of(46);
        public static final Distance kL4CoralHeight = Inches.of(71);  // was 70

        public static final Distance kL1AlgaeHeight = Inches.of(24);
        public static final Distance kL2AlgaeHeight = Inches.of(39);
        public static final Distance kL3AlgaeHeight = Inches.of(54);  

        public static final Distance kL4AlgaeWindupHeight = Inches.of(71);  

        public static final Distance kSafeHomeHeight = Inches.of(19);
        
        public static final LinearVelocity kElevatorMaxVelocity = MetersPerSecond.of(2.0);  // MPS
		public static final LinearAcceleration kElevatorMaxAcceleration = MetersPerSecondPerSecond.of(4.0); // MPSS  was 6
	
        public static final double kElevatorEncoderPositionConversionFactor = kRelativeEncoderScaleRevToMeters; 
        public static final double kElevatorEncoderVelocityConversionFactor = kRelativeEncoderScaleRevToMeters; 
    }

    public class DriverConstants{
        // driver
        public static final Dimensionless kMaxDriveSpeedPercentage = Percent.of(85); // 85%
        public static final Dimensionless kMaxTurnSpeedPercentage  = Percent.of(90); // 90%

        public static final Dimensionless kDriveDeadband = Percent.of(8); // 0.08% deadband
        public static final Dimensionless kRotationDeadband = Percent.of(8); // 0.08% deadband

        //Co-Pilot 1
        public static final int reset = 1;

        public static final int l4 = 2;
        public static final int l3 = 3;
        public static final int l2 = 4;
        public static final int l1 = 5;

        public static final int home = 6;

        public static final int pose_ija = 7;
        public static final int pose_i = 8;
        public static final int pose_j = 9;
        public static final int pose_kla = 10;
        public static final int pose_k = 11;
        public static final int pose_l = 12;

        //Co-pilot 2
        public static final int pose_h = 1;
        public static final int pose_gha = 2;
        public static final int pose_g = 3;
        public static final int pose_f = 4;
        public static final int pose_efa = 5;
        public static final int pose_e = 6;
        public static final int pose_cda = 7;
        public static final int pose_d = 8;
        public static final int pose_c = 9;
        public static final int pose_b = 10;
        public static final int pose_aba = 11;
        public static final int pose_a = 12;
    }

    public class ApproachConstants {
        public static final LinearVelocity maxApproachLinearVelocity = MetersPerSecond.of(2.0);
        public static final LinearAcceleration maxApproachLinearAcceleration = MetersPerSecondPerSecond.of(1.5);
        public static final AngularVelocity maxApproachAngularVelocity = RadiansPerSecond.of(2 * Math.PI);
        public static final AngularAcceleration maxApproachAngularAcceleration = RadiansPerSecondPerSecond.of(4 * Math.PI);
    }
}
