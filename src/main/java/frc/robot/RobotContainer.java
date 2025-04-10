// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Driver;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HomeElevatorCmd;
import frc.robot.commands.JustIntakeCmd;
import frc.robot.commands.TriggerEventCmd;
import frc.robot.commands.WaitForTowerStateCmd;
import frc.robot.commands.WaitToSeeCoralCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Globals;
import frc.robot.subsystems.approach.ApproachSubsystem;
import frc.robot.subsystems.approach.ApproachTarget;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSparkFlex;
import frc.robot.subsystems.elevator.ElevatorIOSparkSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.led.LEDIOReal;
import frc.robot.subsystems.led.LEDIOSim;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.tower.TowerEvent;
import frc.robot.subsystems.tower.TowerState;
import frc.robot.subsystems.tower.TowerSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.wrist.AngleIOSparkFlex;
import frc.robot.subsystems.wrist.IntakeIOSparkFlex;
import frc.robot.subsystems.wrist.SensorIOPWF;
import frc.robot.subsystems.wrist.WristSubsystem;

public class RobotContainer {
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandJoystick       copilot_1 = new CommandJoystick(1);
    private final CommandJoystick       copilot_2 = new CommandJoystick(2);

    static final Transform3d robotToLeftCam = new Transform3d(new Translation3d(0.24, 0.27, 0.21), 
                                                            new Rotation3d(0, Math.toRadians(-5), Math.toRadians(-45)));
    static final Vector<N3> leftCamStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5));
  
    static final Transform3d robotToRightCam = new Transform3d(new Translation3d(0.24, -0.27, 0.217), 
                                                          new Rotation3d(0, Math.toRadians(-5), Math.toRadians(45)));
    static final Vector<N3> rightCamStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5));

    // Instanciate subsystems
    public Drive drive;
    public final Globals globals = new Globals();
    public ElevatorSubsystem elevator;
    public WristSubsystem wrist;
    public TowerSubsystem tower = new TowerSubsystem(elevator, wrist, joystick);
    public VisionSubsystem leftVision = new VisionSubsystem(drive, "LEFT_CAM", robotToLeftCam, leftCamStdDevs);
    public VisionSubsystem rightVision = new VisionSubsystem(drive, "RIGHT_CAM", robotToRightCam, rightCamStdDevs);
    public ApproachSubsystem approach;
    public LEDSubsystem led;

    /* Path follower */
    private final LoggedDashboardChooser<Command> autoChooser;

    /* Intake and Goto Commands */
    private final JustIntakeCmd intakeAndGotoL1 = new JustIntakeCmd(tower, TowerEvent.GOTO_L1);
    private final JustIntakeCmd intakeAndGotoL2 = new JustIntakeCmd(tower, TowerEvent.GOTO_L2);
    private final JustIntakeCmd intakeAndGotoL3 = new JustIntakeCmd(tower, TowerEvent.GOTO_L3);
    private final JustIntakeCmd intakeAndGotoL4 = new JustIntakeCmd(tower, TowerEvent.GOTO_L4);    
    private final WaitToSeeCoralCmd waitToSeeCoral  = new WaitToSeeCoralCmd(tower);

    /* Event Trigger Commands */
    private final TriggerEventCmd intakeCoral = new TriggerEventCmd(tower, TowerEvent.INTAKE_CORAL);
    private final TriggerEventCmd score = new TriggerEventCmd(tower, TowerEvent.SCORE);
    private final TriggerEventCmd intakeLowAlgae = new TriggerEventCmd(tower, TowerEvent.INTAKE_LOW_ALGAE);
    private final TriggerEventCmd intakeHighAlgae = new TriggerEventCmd(tower, TowerEvent.INTAKE_HIGH_ALGAE);
    private final TriggerEventCmd gotoL1 = new TriggerEventCmd(tower, TowerEvent.GOTO_L1);
    private final TriggerEventCmd gotoL3 = new TriggerEventCmd(tower, TowerEvent.GOTO_L3);
    private final TriggerEventCmd gotoL4 = new TriggerEventCmd(tower, TowerEvent.GOTO_L4);

    /* Waiting Commands */
    private final WaitForTowerStateCmd waitForAlgae = new WaitForTowerStateCmd(tower, TowerState.WAITING_FOR_ALGAE);
    private final WaitForTowerStateCmd waitForLowering = new WaitForTowerStateCmd(tower, TowerState.PAUSING_AFTER_SCORING_CORAL);
    private final WaitForTowerStateCmd waitForHome = new WaitForTowerStateCmd(tower, TowerState.HOME);

    /* Home Elevator Command */
    private final HomeElevatorCmd homeElevator = new HomeElevatorCmd(elevator, tower);

    /* Instant Commands */
    private final Command scoreInstant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.SCORE));
    private final Command intakeCoralInstant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_CORAL));

    private final Command intakeLowAlgaeInstant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_LOW_ALGAE));
    private final Command intakeHighAlgaeInstant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_HIGH_ALGAE));

    private final Command seedFieldCentricInstant = drive.runOnce(() -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()))).ignoringDisable(true);
    private final Command stopDrivetrainInstant = drive.runOnce(drive::stop);
    
    private final Command enableSafetyOverrideInstant = leftVision.runOnce(() -> leftVision.setSafetyOverride(true))
                                                        .andThen(rightVision.runOnce(() -> rightVision.setSafetyOverride(true)));
    private final Command enableDirectToAlgaeInstant = tower.runOnce(() -> tower.enableGoToDirectAlgae());

    private final Command homeTowerInstant = tower.runOnce(() -> tower.homeTower());
    private final Command tiltTowerInstant = tower.runOnce(() -> tower.tiltForward());

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
    private final Command robotCentricForward = drive.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0.75, 0, 0)));
    private final Command robotCentricBackward = drive.runOnce(() -> drive.runVelocity(new ChassisSpeeds(-0.75, 0, 0)));
    private final Command robotCentricLeft = drive.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0.25, 0)));
    private final Command robotCentricRight = drive.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, -0.25, 0)));

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive =
                    new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                approach = 
                    new ApproachSubsystem(drive);
                elevator = 
                    new ElevatorSubsystem(
                        new ElevatorIOSparkFlex());
                led = 
                    new LEDSubsystem(
                        new LEDIOReal(0, 25));
                wrist = 
                    new WristSubsystem(
                        new AngleIOSparkFlex(),
                        new IntakeIOSparkFlex(),
                        new SensorIOPWF());
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
                approach = 
                    new ApproachSubsystem(drive);
                elevator = 
                    new ElevatorSubsystem(
                        new ElevatorIOSparkSim());
                led = 
                    new LEDSubsystem(
                        new LEDIOSim());
                break;
            default:
                // Replayed robot, disable IO implementations
                drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});
                approach = 
                    new ApproachSubsystem(drive);
                elevator = 
                    new ElevatorSubsystem(
                        new ElevatorIO() {});
                led = 
                    new LEDSubsystem(
                        new LEDIO() {});
                break;
        }
        
        // All named commands =========================
        NamedCommands.registerCommand("INTAKE_CORAL",              intakeCoral);
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L1",        intakeAndGotoL1);
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L2",        intakeAndGotoL2);
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L3",        intakeAndGotoL3);
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L4",        intakeAndGotoL4);
        NamedCommands.registerCommand("WAIT_TO_SEE_CORAL",         waitToSeeCoral);  //Does not set height
        
        NamedCommands.registerCommand("SCORE_CORAL",               score);
        NamedCommands.registerCommand("SCORE_ALGAE",               score); // same as coral
        NamedCommands.registerCommand("GET_ALGAE",                 intakeHighAlgae);
        NamedCommands.registerCommand("GET_LOW_ALGAE",             intakeLowAlgae);
        NamedCommands.registerCommand("GO_TO_L1",                  gotoL1);
        NamedCommands.registerCommand("WAIT_FOR_ALGAE",            waitForAlgae);
        NamedCommands.registerCommand("WAIT_FOR_LOWERING",         waitForLowering);
        NamedCommands.registerCommand("WAIT_FOR_HOME",             waitForHome);
        NamedCommands.registerCommand("GO_DIRECTLY_TO_ALGAE",      enableDirectToAlgaeInstant);
    
        // All Path Planner event triggers  ===========
        new EventTrigger("INTAKE_CORAL").onTrue(intakeCoral);
        new EventTrigger("INTAKE_AND_GOTO_L3").onTrue( intakeAndGotoL3);
        new EventTrigger("INTAKE_AND_GOTO_L4").onTrue(intakeAndGotoL4);
        new EventTrigger("GOTO_L1_ALGAE").onTrue(gotoL1);
        new EventTrigger("GOTO_L3_ALGAE").onTrue(gotoL3);
        new EventTrigger("GOTO_L4_ALGAE").onTrue(gotoL4);
        new EventTrigger("INTAKE_LOW_ALGAE").onTrue(intakeLowAlgae);
        new EventTrigger("INTAKE_HIGH_ALGAE").onTrue(intakeHighAlgae);
 
        // Configure Auto Chooser  ===============================
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
            "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  
        configureBindings();

        // avoid the PathPlanner startup delay....
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive, 
                () -> -joystick.getLeftY() * Constants.Drivetrain.kMaxVelocityMPS * Constants.Driver.kMaxDriveSpeed * tower.getTowerSpeedSafetyFactor(), 
                () -> -joystick.getLeftX() * Constants.Drivetrain.kMaxVelocityMPS * Constants.Driver.kMaxDriveSpeed  * tower.getTowerSpeedSafetyFactor(), 
                () -> -joystick.getRightX() * Constants.Drivetrain.kMaxAngularVelocityRPS * Constants.Driver.kMaxTurnSpeed * tower.getTowerSpeedSafetyFactor()
            ));

        // Driver Buttons
        joystick.rightTrigger(0.5).onTrue(scoreInstant);  // score coral or algae

        joystick.back().onTrue(seedFieldCentricInstant);  // reset field centric home

        joystick.start().onTrue(homeElevator);  //home the elevator
        joystick.rightStick().onTrue(tiltTowerInstant); // Tilt the elevator

        // Change to .toggleOnTrue to make it toggle on/off when the button is pressed
        joystick.leftBumper().onTrue(intakeCoralInstant)
            .whileTrue(DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -joystick.getLeftY() * Constants.Drivetrain.kMaxVelocityMPS * Constants.Driver.kMaxDriveSpeed * tower.getTowerSpeedSafetyFactor(), 
                () -> -joystick.getLeftX() * Constants.Drivetrain.kMaxVelocityMPS * Constants.Driver.kMaxDriveSpeed  * tower.getTowerSpeedSafetyFactor(), 
                () -> Constants.kFieldLayout.getTagPose(13).get().getRotation().toRotation2d()
            ));  // collect coral left side

        joystick.rightBumper().onTrue(intakeCoralInstant)
            .whileTrue(DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -joystick.getLeftY() * Constants.Drivetrain.kMaxVelocityMPS * Constants.Driver.kMaxDriveSpeed * tower.getTowerSpeedSafetyFactor(), 
                () -> -joystick.getLeftX() * Constants.Drivetrain.kMaxVelocityMPS * Constants.Driver.kMaxDriveSpeed  * tower.getTowerSpeedSafetyFactor(), 
                () -> Constants.kFieldLayout.getTagPose(12).get().getRotation().toRotation2d()
            ));  // collect coral left side

        joystick.y().onTrue(intakeHighAlgaeInstant);
        joystick.a().onTrue(intakeLowAlgaeInstant);

        joystick.x().onTrue(approachBargeInstant);
        joystick.b().onTrue(approachProcessorInstant);
        
        // ==== Approach Buttons ================================

        joystick.leftTrigger(0.5).onTrue(startApproachInstant)
        .onFalse(stopDrivetrainInstant);

        // ==== NON Field Centric driving ================================

        joystick.pov(0).whileTrue(robotCentricForward);
        joystick.pov(180).whileTrue(robotCentricBackward);
        joystick.pov(90).whileTrue(robotCentricRight);
        joystick.pov(270).whileTrue(robotCentricLeft);
            
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
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.get();
    }
}
