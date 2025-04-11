// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

public class Constants {
    @AutoLogOutput
    public static final Mode simMode = Mode.SIM;

    @AutoLogOutput
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    @AutoLogOutput
    public static final double kDt = 0.02;

    @AutoLogOutput
    public static final AprilTagFields kField = AprilTagFields.k2025ReefscapeAndyMark;
    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(kField);

    public class Wrist {
        @AutoLogOutput
        public static final int kAngleMotorId = 61;

        @AutoLogOutput
        public static final int kIntakeMotorId = 62;

        @AutoLogOutput
        public static final int kExitTOFId = 63;

        @AutoLogOutput
        public static final int kEnterTOFId = 64;

        @AutoLogOutput
        public static final int kAngleCurrentLimit = 50;

        @AutoLogOutput
        public static final int kIntakeCurrentLimit = 50;

        @AutoLogOutput
		public static final int kAlgaeGrabbedCurrent = 40;

        @AutoLogOutput
        public static final double kAngleFactor = 360 * 24 / 40; // 216 degrees

        @AutoLogOutput
        public static final double kAngleGearing = 40.0;

        @AutoLogOutput
        public static final double kP = 0.04; // was 0.02

        @AutoLogOutput
        public static final double kI = 0;

        @AutoLogOutput
        public static final double kD = 0.0;

        @AutoLogOutput
        public static final double kAngleTollerance = 2;

		@AutoLogOutput
        public static final double kAnglePower = 0.5; // was 1.0

		@AutoLogOutput
        public static final double kCoralSlowIntakePower    = -0.1;

        @AutoLogOutput
		public static final double kCoralL1ScoringPower     = -0.2;

        @AutoLogOutput
        public static final double kCoralIntakePower        = -0.3;
        
        @AutoLogOutput
		public static final double kCoralL23ScoringPower    = -0.30;  // was -1.0

        @AutoLogOutput
        public static final double kCoralL4ScoringPower     = -0.80;  // was -1.0

        @AutoLogOutput
        public static final double kCoralFeedPower          = -0.04;

        @AutoLogOutput
        public static final double kCoralHoldPower          =  0.04;
        
        @AutoLogOutput
  		public static final double kAlgaeIntakePower        =  0.3;

        @AutoLogOutput
        public static final double kAlgaeScoringPower       = -1.0;

        @AutoLogOutput
        public static final double kAngleMaxVelocityDPS       = 400; 

        @AutoLogOutput 
		public static final double kAngleMaxAccelerationDPSPS = 1000; 

        @AutoLogOutput
        public static final double kIntakeAngleDegrees             = 3;

        @AutoLogOutput
        public static final double kSafeAngleDegrees               = 30;

        @AutoLogOutput
        public static final double kL4AngleDegrees                 = 50; // was 48

        @AutoLogOutput
        public static final double kHighAlgaeAngleDegrees          = 60;

        @AutoLogOutput
        public static final double kAlgaeIntakeAngleDegrees        = 180; 

        @AutoLogOutput
        public static final double kAlgaeWindupAngleDegrees        = 150; 

        @AutoLogOutput
        public static final double kAlgaeReleaseAngleDegrees       = 125;

        @AutoLogOutput
        public static final double kAlgaeBackspinDegrees           = 95;

        @AutoLogOutput
        public static final double kAlgaeReleaseGoalAngleDegrees   = 22; // was 20

        @AutoLogOutput
        public static final double kMaxAngleWhenHomeDegrees        = 182;

        @AutoLogOutput
        public static final double kMaxAngleDegrees                = 210;

        @AutoLogOutput
        public static final double kMaxCoralDetectRangeMM   = 80;

        @AutoLogOutput
        public static final double kTOFSampleTime           = 24;
    }

    public class Elevator {
        @AutoLogOutput
        public static final double elevatorHomeHeightMeters = Units.inchesToMeters(17.5);  // only valid when elevator is homed;

        // scale factors
        @AutoLogOutput
        public static final double kRelativeEncoderScaleRevToMeters = 0.0315;  // 25 turns for 31 inches of travel

        @AutoLogOutput
        public static final double kAbsoluteEncoderScaleVoltsToMeters = 0.498;

        @AutoLogOutput
        public static final double kAbsoluteEncoderOffsetVoltsToMeters = 0.43;

        @AutoLogOutput
        public static final double kP = 6;  // was 8

        @AutoLogOutput
        public static final double kI = 0;

        @AutoLogOutput
        public static final double kD = 0.46;

        @AutoLogOutput
        public static final double kS = 0.04435;    //  these may need to be rescaled for meters by dividing 

        @AutoLogOutput
		public static final double kG = 0.257;      //  by kRelativeEncoderScaleRevToMeters

        @AutoLogOutput
		public static final double kV = 0.117 ;     //

        @AutoLogOutput
		public static final double kA = 0.00803 ;   //

        @AutoLogOutput
        public static final double kHeightTolleranceMeters = Units.inchesToMeters(1.0);

        @AutoLogOutput
        public static final int kElevatorCurrentLimit = 60;

        @AutoLogOutput
        public static final int kElevatorMotorLeftId = 51;

        @AutoLogOutput
        public static final int kElevatorMotorCenterId = 52;

        @AutoLogOutput
        public static final int kElevatorMotorRightId = 53;

        @AutoLogOutput
        public static final double kElevatorMaxHeightMeters = Units.inchesToMeters(73);

        @AutoLogOutput
        public static final double kElevatorSpeedSafeHeightMeters = Units.inchesToMeters(40);

        @AutoLogOutput
        public static final double kElevatorMinHeightMeters = Units.inchesToMeters(17.5);

        @AutoLogOutput
        public static final double kIntakeHeightMeters = Units.inchesToMeters(17.5); 
        
        @AutoLogOutput
        public static final double kL1CoralHeightMeters = Units.inchesToMeters(21);

        @AutoLogOutput
        public static final double kL2CoralHeightMeters = Units.inchesToMeters(31);

        @AutoLogOutput
        public static final double kL3CoralHeightMeters = Units.inchesToMeters(45);  // was 46

        @AutoLogOutput
        public static final double kL4CoralHeightMeters = Units.inchesToMeters(70);  

        @AutoLogOutput
        public static final double kL1AlgaeHeightMeters = Units.inchesToMeters(22);  // was 24

        @AutoLogOutput
        public static final double kL2AlgaeHeightMeters = Units.inchesToMeters(39);

        @AutoLogOutput
        public static final double kL3AlgaeHeightMeters = Units.inchesToMeters(54);  

        @AutoLogOutput
        public static final double kL4AlgaeWindupHeightMeters = Units.inchesToMeters(72); 

        @AutoLogOutput
        public static final double kSafeHomeHeightMeters = Units.inchesToMeters(19);
        
        //public static final double kElevatorMaxVelocityMPS = 2.0;  // MPS
        @AutoLogOutput
		public static final double kElevatorMaxAccelerationMPSPS = 4.0; // MPSS  was 6

        @AutoLogOutput
        public static final double kElevatorEncoderPositionConversionFactor = kRelativeEncoderScaleRevToMeters; 

        @AutoLogOutput
        public static final double kElevatorEncoderVelocityConversionFactor = kRelativeEncoderScaleRevToMeters / 60.0; 
    }

    public class Driver{
        // driver 
        @AutoLogOutput
        public static final double kMaxDriveSpeed = 1.0; // Percent of kMaxVelocityMPS

        @AutoLogOutput
        public static final double kMaxTurnSpeed  = 0.85; // Percent of kMaxAngularVelocityRPS

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

    public class Drivetrain {
        @AutoLogOutput
        public static final double kMaxVelocityMPS = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        @AutoLogOutput
        public static final double kMaxAccelerationMPSPS = 2 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        @AutoLogOutput
        public static final double kMaxAngularVelocityRPS = 2 * Math.PI;

        @AutoLogOutput
        public static final double kMaxAngularAccelerationRPSPS = 4 * Math.PI;

        @AutoLogOutput
        public static final double kPHeading = 10.0;

        @AutoLogOutput
        public static final double kIHeading = 0.0;

        @AutoLogOutput
        public static final double kDHeading = 0.0;
    }

    public class Approach {
        @AutoLogOutput
		public static final double maxApproachLinearVelocityPercent = 0.6; // Was 2.0m/s, now percent of kMaxVelocityMPS

        @AutoLogOutput
		public static final double maxApproachLinearAccelerationPercent = 0.2; // Was 1.5m/s, now percent of kMaxAccelerationMPSPS

        @AutoLogOutput
		public static final double maxApproachAngularVelocityPercent = 1.0; // Was 2PI, now percent of kMaxAngularVelocityRPS

        @AutoLogOutput
		public static final double maxApproachAngularAccelerationPercent = 1.0; // Was 4PI, now percent of kMaxAngularAccelerationRPSPS
	}

    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public class Vision {
        @AutoLogOutput
        public static final String camera0Name = "LEFT_CAM";

        @AutoLogOutput
        public static final String camera1Name = "RIGHT_CAM";

        @AutoLogOutput
        public static final Transform3d robotToCamera0 = new Transform3d(new Translation3d(0.24, 0.27, 0.21), 
                                                            new Rotation3d(0, Math.toRadians(-5), Math.toRadians(-45)));

        @AutoLogOutput
        public static final Vector<N3> camera0StdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5));

        @AutoLogOutput
        public static final Transform3d robotToCamera1 = new Transform3d(new Translation3d(0.24, -0.27, 0.217), 
                                                          new Rotation3d(0, Math.toRadians(-5), Math.toRadians(45)));

        @AutoLogOutput
        public static final Vector<N3> camera1StdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5));

        @AutoLogOutput
        public static double maxAmbiguity = 0.3;

        @AutoLogOutput
        public static double maxZError = 0.75;

        @AutoLogOutput
        public static double linearStdDevBaseline = 0.02; // Meters

        @AutoLogOutput
        public static double angularStdDevBaseline = 0.06; // Radians

        @AutoLogOutput
        public static double[] cameraStdDevFactors = new double[] {
                1.0, // Camera 0
                1.0 // Camera 1
        };

        @AutoLogOutput
        public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
        
        @AutoLogOutput
        public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
    }
}
