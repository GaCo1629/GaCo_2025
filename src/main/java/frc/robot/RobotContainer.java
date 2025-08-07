// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.HomeElevatorCmd;
import frc.robot.commands.JustIntakeCmd;
import frc.robot.commands.TriggerEventCmd;
import frc.robot.commands.WaitForTowerStateCmd;
import frc.robot.commands.ScoreAndGotoAlgaeLevel;
import frc.robot.commands.WaitToSeeCoralCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ApproachSubsystem;
import frc.robot.subsystems.ApproachTarget;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Globals;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TowerEvent;
import frc.robot.subsystems.TowerState;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.Constants.Driver;

public class RobotContainer {
    //public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    //public static final double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // was 0.75

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drivetrain.kMaxVelocityMPS * 0.08).withRotationalDeadband(Constants.Drivetrain.kMaxAngularVelocityRPS * 0.08) // Add a 8% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use closed-loop control for drive motors

    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity); // Use closed-loop control for drive motors

    private final SwerveRequest.FieldCentricFacingAngle rotateTo = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(Constants.Drivetrain.kMaxVelocityMPS * 0.08)
            .withRotationalDeadband(Constants.Drivetrain.kMaxAngularVelocityRPS * 0.08)
            .withDriveRequestType(DriveRequestType.Velocity) // Use closed-loop control for drive motors
            .withHeadingPID(Constants.Drivetrain.kPHeading, Constants.Drivetrain.kIHeading, Constants.Drivetrain.kDHeading);

    private final Telemetry logger = new Telemetry(Constants.Drivetrain.kMaxVelocityMPS);

    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandJoystick       copilot_1 = new CommandJoystick(1);
    private final CommandJoystick       copilot_2 = new CommandJoystick(2);

    static final Transform3d robotToLeftCam = new Transform3d(new Translation3d(0.24, 0.27, 0.21), 
                                                            new Rotation3d(0, Math.toRadians(-5), Math.toRadians(-45)));
    static final Vector<N3> leftCamStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5));
  
    static final Transform3d robotToRightCam = new Transform3d(new Translation3d(0.24, -0.27, 0.217), 
                                                          new Rotation3d(0, Math.toRadians(-5), Math.toRadians(45)));
    static final Vector<N3> rightCamStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5));

    // Instanciate subsystems
    public final Globals globals = new Globals();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final WristSubsystem wrist = new WristSubsystem();
    public final TowerSubsystem tower = new TowerSubsystem(elevator, wrist, pilot);
    public final VisionSubsystem leftVision = new VisionSubsystem(drivetrain, "LEFT_CAM", robotToLeftCam, leftCamStdDevs);
    public final VisionSubsystem rightVision = new VisionSubsystem(drivetrain, "RIGHT_CAM", robotToRightCam, rightCamStdDevs);
    public final ApproachSubsystem approach = new ApproachSubsystem(drivetrain);
    public final LEDSubsystem led = new LEDSubsystem(0);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /* Intake and Goto Commands */
    private final JustIntakeCmd intakeAndGotoL1 = new JustIntakeCmd(tower, TowerEvent.GOTO_L1);
    private final JustIntakeCmd intakeAndGotoL2 = new JustIntakeCmd(tower, TowerEvent.GOTO_L2);
    private final JustIntakeCmd intakeAndGotoL3 = new JustIntakeCmd(tower, TowerEvent.GOTO_L3);
    private final JustIntakeCmd intakeAndGotoL4 = new JustIntakeCmd(tower, TowerEvent.GOTO_L4);    
    private final ScoreAndGotoAlgaeLevel scoreAndGotoAlgaeL2 = new ScoreAndGotoAlgaeLevel(tower, 2);
    private final ScoreAndGotoAlgaeLevel scoreAndGotoAlgaeL3 = new ScoreAndGotoAlgaeLevel(tower, 3);
    private final WaitToSeeCoralCmd waitToSeeCoral  = new WaitToSeeCoralCmd(tower);

    /* Event Trigger Commands */
    private final TriggerEventCmd score = new TriggerEventCmd(tower, TowerEvent.SCORE);
    private final TriggerEventCmd gotoL1 = new TriggerEventCmd(tower, TowerEvent.GOTO_L1);
    private final TriggerEventCmd gotoL4 = new TriggerEventCmd(tower, TowerEvent.GOTO_L4);

    /* Waiting Commands */
    private final WaitForTowerStateCmd waitForAlgae = new WaitForTowerStateCmd(tower, TowerState.WAITING_FOR_ALGAE);
    private final WaitForTowerStateCmd waitForLowering = new WaitForTowerStateCmd(tower, TowerState.FINISHING_SCORING_CORAL);
    private final WaitForTowerStateCmd waitForChangingAlgaeHeight = new WaitForTowerStateCmd(tower, TowerState.CHANGING_ALGAE_HEIGHT);

    /* Home Elevator Command */
    private final HomeElevatorCmd homeElevator = new HomeElevatorCmd(elevator, tower);

    /* Instant Commands */
    private final Command scoreInstant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.SCORE));
    private final Command intakeCoralInstant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_CORAL));

    private final Command intakeLowAlgaeInstant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_LOW_ALGAE));
    private final Command intakeHighAlgaeInstant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_HIGH_ALGAE));

    private final Command seedFieldCentricInstant = drivetrain.runOnce(() -> drivetrain.seedFieldCentric());
    private final Command stopDrivetrainInstant = drivetrain.runOnce(() -> drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.0).withVelocityY(0.0)));
    
    private final Command enableSafetyOverrideInstant = leftVision.runOnce(() -> leftVision.setSafetyOverride(true))
                                                        .andThen(rightVision.runOnce(() -> rightVision.setSafetyOverride(true)));

    private final Command homeTowerInstant = tower.runOnce(() -> tower.homeTower());
    private final Command tiltWristNearInstant = tower.runOnce(() -> tower.forceWristTilt(Constants.Wrist.kSafeAngleDegrees));
    private final Command tiltWristFarInstant = tower.runOnce(() -> tower.forceWristTilt(Constants.Wrist.kAlgaeReleaseAngleDegrees));

    private final Command gotoL1Instant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L1));
    private final Command gotoL2Instant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L2));
    private final Command gotoL3Instant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L3));
    private final Command gotoL4Instant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L4));

    private final Command startApproachInstant = approach.runOnce(() -> approach.startApproach());
    private final Command reefAInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_A));
    private final Command reefBInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_B));
    private final Command reefABInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_AB));
    private final Command reefCInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_C));
    private final Command reefDInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_D));
    private final Command reefCDInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_CD));
    private final Command reefEInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_E));
    private final Command reefFInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_F));
    private final Command reefEFInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_EF));
    private final Command reefGInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_G));
    private final Command reefHInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_H));
    private final Command reefGHInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_GH));
    private final Command reefIInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_I));
    private final Command reefJInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_J));
    private final Command reefIJInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_IJ));
    private final Command reefKInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_K));
    private final Command reefLInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_L));
    private final Command reefKLInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_KL));
    private final Command approachBargeInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.BARGE));
    private final Command approachProcessorInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.PROCESSOR));

    /* Robot Centric Movement Commands */
    private final Command robotCentricForward = drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.75).withVelocityY(0));
    private final Command robotCentricBackward = drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.75).withVelocityY(0));
    private final Command robotCentricLeft = drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.0).withVelocityY(0.25));
    private final Command robotCentricRight = drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.0).withVelocityY(-0.25));

    public RobotContainer() {
        
        // All named commands =========================
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L1",        intakeAndGotoL1);  
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L2",        intakeAndGotoL2);    // used
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L3",        intakeAndGotoL3);    // used
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L4",        intakeAndGotoL4);
        NamedCommands.registerCommand("WAIT_TO_SEE_CORAL",         waitToSeeCoral);     // used
        
        NamedCommands.registerCommand("SCORE",                     score);              // used
        NamedCommands.registerCommand("SCORE_THEN_GOTO_ALGAE_L2",  scoreAndGotoAlgaeL2);// used
        NamedCommands.registerCommand("SCORE_THEN_GOTO_ALGAE_L3",  scoreAndGotoAlgaeL3);// used
        NamedCommands.registerCommand("GOTO_L1",                   gotoL1);
        NamedCommands.registerCommand("WAIT_FOR_ALGAE",            waitForAlgae);       // used
        NamedCommands.registerCommand("WAIT_FOR_LOWERING",         waitForLowering);    // used
        NamedCommands.registerCommand("WAIT_FOR_CHANGING_ALGAE_HEIGHT", waitForChangingAlgaeHeight);

    
        // All Path Planner event triggers  ===========
        new EventTrigger("INTAKE_AND_GOTO_L3").onTrue( intakeAndGotoL3);
        new EventTrigger("INTAKE_AND_GOTO_L4").onTrue(intakeAndGotoL4);
        new EventTrigger("GOTO_L1_ALGAE").onTrue(gotoL1);                               // used
        new EventTrigger("GOTO_L4_ALGAE").onTrue(gotoL4);
        
        // Configure Auto Chooser  ===============================
        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Send drive module data to dashboard
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
              builder.setSmartDashboardType("SwerveDrive");
          
              builder.addDoubleProperty("Front Left Angle", () -> drivetrain.getModule(0).getPosition(false).angle.getRadians(), null);
              builder.addDoubleProperty("Front Left Velocity", () -> drivetrain.getModule(0).getDriveMotor().getVelocity().getValueAsDouble(), null);
          
              builder.addDoubleProperty("Front Right Angle", () -> drivetrain.getModule(1).getPosition(false).angle.getRadians(), null);
              builder.addDoubleProperty("Front Right Velocity", () -> drivetrain.getModule(1).getDriveMotor().getVelocity().getValueAsDouble(), null);
          
              builder.addDoubleProperty("Back Left Angle", () -> drivetrain.getModule(2).getPosition(false).angle.getRadians(), null);
              builder.addDoubleProperty("Back Left Velocity", () -> drivetrain.getModule(2).getDriveMotor().getVelocity().getValueAsDouble(), null);
          
              builder.addDoubleProperty("Back Right Angle", () -> drivetrain.getModule(3).getPosition(false).angle.getRadians(), null);
              builder.addDoubleProperty("Back Right Velocity", () -> drivetrain.getModule(3).getDriveMotor().getVelocity().getValueAsDouble(), null);
          
              builder.addDoubleProperty("Robot Angle", () -> drivetrain.getRotation3d().getMeasureAngle().baseUnitMagnitude(), null);
            }
          });
  
        configureBindings();

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-pilot.getLeftY() * Constants.Drivetrain.kMaxVelocityMPS * Constants.Driver.kMaxDriveSpeed * tower.getTowerSpeedSafetyFactor()) // Drive forward with negative Y (forward)
                    .withVelocityY(-pilot.getLeftX() * Constants.Drivetrain.kMaxVelocityMPS * Constants.Driver.kMaxDriveSpeed  * tower.getTowerSpeedSafetyFactor()) // Drive left with negative X (left)
                    .withRotationalRate(-pilot.getRightX() * Constants.Drivetrain.kMaxAngularVelocityRPS * Constants.Driver.kMaxTurnSpeed * tower.getTowerSpeedSafetyFactor()) // Drive counterclockwise with negative X (left)
            )
        );

        // Change PathPlanner field size for robocon
        FlippingUtil.fieldSizeX = 15.814;


        // avoid the PathPlanner startup delay....
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {

        // Driver Buttons
        pilot.rightTrigger(0.5).onTrue(scoreInstant);  // score coral or algae

        pilot.back().onTrue(seedFieldCentricInstant);  // reset field centric home

        pilot.start().onTrue(homeElevator);  //home the elevator
        pilot.rightStick().onTrue(tiltWristNearInstant); // Tilt the wrist to free coral
        pilot.leftStick().onTrue(tiltWristFarInstant); // Tilt the wrist to free algae

        // Change to .toggleOnTrue to make it toggle on/off when the button is pressed
        pilot.leftBumper().onTrue(intakeCoralInstant)
            .whileTrue(drivetrain.applyRequest(() -> 
                    rotateTo.withTargetDirection(Constants.kFieldLayout.getTagPose(13).get().getRotation().toRotation2d())
                        .withVelocityX(-pilot.getLeftY() * Constants.Drivetrain.kMaxVelocityMPS * Constants.Approach.maxApproachLinearVelocityPercent * 1.25 * tower.getTowerSpeedSafetyFactor()) // was max 2.5m/s
                        .withVelocityY(-pilot.getLeftX() * Constants.Drivetrain.kMaxVelocityMPS * Constants.Approach.maxApproachLinearVelocityPercent * 1.25 * tower.getTowerSpeedSafetyFactor()) // was max 2.5m/s
                        .withMaxAbsRotationalRate(Constants.Drivetrain.kMaxAngularVelocityRPS * Constants.Approach.maxApproachAngularVelocityPercent  * tower.getTowerSpeedSafetyFactor()) // was max 0.75*PI
                    ));  // collect coral left side

        pilot.rightBumper().onTrue(intakeCoralInstant)
            .whileTrue(drivetrain.applyRequest(() -> 
                    rotateTo.withTargetDirection(Constants.kFieldLayout.getTagPose(12).get().getRotation().toRotation2d())
                        .withVelocityX(-pilot.getLeftY() * Constants.Drivetrain.kMaxVelocityMPS * Constants.Approach.maxApproachLinearVelocityPercent * 1.25 * tower.getTowerSpeedSafetyFactor()) // was max 2.5m/s
                        .withVelocityY(-pilot.getLeftX() * Constants.Drivetrain.kMaxVelocityMPS * Constants.Approach.maxApproachLinearVelocityPercent * 1.25 * tower.getTowerSpeedSafetyFactor()) // was max 2.5m/s
                        .withMaxAbsRotationalRate(Constants.Drivetrain.kMaxAngularVelocityRPS * Constants.Approach.maxApproachAngularVelocityPercent  * tower.getTowerSpeedSafetyFactor()) // was max 0.75*PI
                    )); // collect coral right side

        pilot.y().onTrue(intakeHighAlgaeInstant);
        pilot.a().onTrue(intakeLowAlgaeInstant);
        pilot.x().onTrue(approachBargeInstant);
        pilot.b().onTrue(approachProcessorInstant);
        
        // ==== Approach Buttons ================================

        pilot.leftTrigger(0.5).onTrue(startApproachInstant)
                                        .onFalse(stopDrivetrainInstant);

        // ==== NON Field Centric driving ================================

        pilot.pov(0).whileTrue(robotCentricForward);
        pilot.pov(180).whileTrue(robotCentricBackward);
        pilot.pov(90).whileTrue(robotCentricRight);
        pilot.pov(270).whileTrue(robotCentricLeft);
            
        // ====  CoPilot 1 Buttons  ======================================

        copilot_1.button(Driver.reset).onTrue(enableSafetyOverrideInstant );
        copilot_1.button(Driver.home).onTrue(homeTowerInstant);

        copilot_1.button(Driver.l1).onTrue(gotoL1Instant);
        copilot_1.button(Driver.l2).onTrue(gotoL2Instant);
        copilot_1.button(Driver.l3).onTrue(gotoL3Instant);
        copilot_1.button(Driver.l4).onTrue(gotoL4Instant);

        copilot_1.button(Driver.pose_i).onTrue(reefIInstant);
        copilot_1.button(Driver.pose_j).onTrue(reefJInstant);
        copilot_1.button(Driver.pose_ija).onTrue(reefIJInstant);
        copilot_1.button(Driver.pose_k).onTrue(reefKInstant);
        copilot_1.button(Driver.pose_l).onTrue(reefLInstant);
        copilot_1.button(Driver.pose_kla).onTrue(reefKLInstant);
        
        // ===  CoPilot 2 Buttons  ===========================================
        copilot_2.button(Driver.pose_a).onTrue(reefAInstant);
        copilot_2.button(Driver.pose_b).onTrue(reefBInstant);
        copilot_2.button(Driver.pose_aba).onTrue(reefABInstant);
        copilot_2.button(Driver.pose_c).onTrue(reefCInstant);
        copilot_2.button(Driver.pose_d).onTrue(reefDInstant);
        copilot_2.button(Driver.pose_cda).onTrue(reefCDInstant);
        copilot_2.button(Driver.pose_e).onTrue(reefEInstant);
        copilot_2.button(Driver.pose_f).onTrue(reefFInstant);
        copilot_2.button(Driver.pose_efa).onTrue(reefEFInstant);
        copilot_2.button(Driver.pose_g).onTrue(reefGInstant);
        copilot_2.button(Driver.pose_h).onTrue(reefHInstant);
        copilot_2.button(Driver.pose_gha).onTrue(reefGHInstant);
       
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
