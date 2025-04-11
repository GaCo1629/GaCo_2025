// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.subsystems.approach.Approach;
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
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.led.LEDIOReal;
import frc.robot.subsystems.led.LEDIOSim;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.tower.TowerEvent;
import frc.robot.subsystems.tower.TowerState;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.AngleIO;
import frc.robot.subsystems.wrist.AngleIOSparkFlex;
import frc.robot.subsystems.wrist.AngleIOSparkSim;
import frc.robot.subsystems.wrist.IntakeIO;
import frc.robot.subsystems.wrist.IntakeIOSparkFlex;
import frc.robot.subsystems.wrist.IntakeIOSparkSim;
import frc.robot.subsystems.wrist.SensorIO;
import frc.robot.subsystems.wrist.SensorIOPWF;
import frc.robot.subsystems.wrist.SensorIOSim;
import frc.robot.subsystems.wrist.Wrist;

public class RobotContainer {
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandJoystick       copilot_1 = new CommandJoystick(1);
    private final CommandJoystick       copilot_2 = new CommandJoystick(2);

    // Instanciate subsystems
    public Approach approach;
    public Drive drive;
    public Elevator elevator;
    public Globals globals;
    public LED led;
    public Tower tower;
    public Vision vision;
    public Wrist wrist;

    /* Path follower */
    private final LoggedDashboardChooser<Command> autoChooser;

    /* Intake and Goto Commands */
    private JustIntakeCmd intakeAndGotoL1;
    private JustIntakeCmd intakeAndGotoL2;
    private JustIntakeCmd intakeAndGotoL3;
    private JustIntakeCmd intakeAndGotoL4;    
    private WaitToSeeCoralCmd waitToSeeCoral;

    /* Event Trigger Commands */
    private TriggerEventCmd intakeCoral;
    private TriggerEventCmd score;
    private TriggerEventCmd intakeLowAlgae;
    private TriggerEventCmd intakeHighAlgae;
    private TriggerEventCmd gotoL1;
    private TriggerEventCmd gotoL3;
    private TriggerEventCmd gotoL4;

    /* Waiting Commands */
    private WaitForTowerStateCmd waitForAlgae;
    private WaitForTowerStateCmd waitForLowering;
    private WaitForTowerStateCmd waitForHome;

    /* Home Elevator Command */
    private HomeElevatorCmd homeElevator;

    /* Instant Commands */
    private Command scoreInstant;
    private Command intakeCoralInstant;

    private Command intakeLowAlgaeInstant;
    private Command intakeHighAlgaeInstant;

    private Command seedFieldCentricInstant;
    private Command stopDrivetrainInstant;
    
    private Command enableSafetyOverrideInstant;
    //                                                    .andThen(rightVision.runOnce(() -> rightVision.setSafetyOverride(true)));
    private Command enableDirectToAlgaeInstant;

    private Command homeTowerInstant;
    private Command tiltTowerInstant;

    private Command gotoL1Instant;
    private Command gotoL2Instant;
    private Command gotoL3Instant;
    private Command gotoL4Instant;

    private Command startApproachInstant;
    private Command reefAInstant;
    private Command reefBInstant;
    private Command reefABInstant;
    private Command reefCInstant;
    private Command reefDInstant;
    private Command reefCDInstant;
    private Command reefEInstant;
    private Command reefFInstant;
    private Command reefEFInstant;
    private Command reefGInstant;
    private Command reefHInstant;
    private Command reefGHInstant;
    private Command reefIInstant;
    private Command reefJInstant;
    private Command reefIJInstant;
    private Command reefKInstant;
    private Command reefLInstant;
    private Command reefKLInstant;
    private Command approachBargeInstant;
    private Command approachProcessorInstant;

    /* Robot Centric Movement Commands */
    private Command robotCentricForward;
    private Command robotCentricBackward;
    private Command robotCentricLeft;
    private Command robotCentricRight;

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
                elevator = 
                    new Elevator(
                        new ElevatorIOSparkFlex());
                led = 
                    new LED(
                        new LEDIOReal(0, 25));
                vision = 
                    new Vision(
                        drive::addVisionMeasurement, 
                        new VisionIOPhotonVision(Constants.Vision.camera0Name, Constants.Vision.robotToCamera0),
                        new VisionIOPhotonVision(Constants.Vision.camera1Name, Constants.Vision.robotToCamera1));
                wrist = 
                    new Wrist(
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
                elevator = 
                    new Elevator(
                        new ElevatorIOSparkSim(
                            new ElevatorIOSparkFlex()));
                led = 
                    new LED(
                        new LEDIOSim(0, 25));
                vision = 
                    new Vision(
                        drive::addVisionMeasurement, 
                        new VisionIOPhotonVisionSim(Constants.Vision.camera0Name, Constants.Vision.robotToCamera0, drive::getPose),
                        new VisionIOPhotonVisionSim(Constants.Vision.camera1Name, Constants.Vision.robotToCamera1, drive::getPose));
                wrist = 
                    new Wrist(
                        new AngleIOSparkSim(
                            new AngleIOSparkFlex()), 
                        new IntakeIOSparkSim(
                            new IntakeIOSparkFlex()), 
                        new SensorIOSim());
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
                elevator = 
                    new Elevator(
                        new ElevatorIO() {});
                led = 
                    new LED(
                        new LEDIO() {});
                vision = 
                    new Vision(
                        drive::addVisionMeasurement, 
                        new VisionIO() {},
                        new VisionIO() {});
                wrist = 
                    new Wrist(
                        new AngleIO() {}, 
                        new IntakeIO() {}, 
                        new SensorIO() {});
                break;
        }
        approach = new Approach(drive);
        globals = new Globals();
        tower = new Tower(elevator, wrist, joystick);

        instantiateCommands();
        
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

    private void instantiateCommands() {
        /* Intake and Goto Commands */
        intakeAndGotoL1 = new JustIntakeCmd(tower, TowerEvent.GOTO_L1);
        intakeAndGotoL2 = new JustIntakeCmd(tower, TowerEvent.GOTO_L2);
        intakeAndGotoL3 = new JustIntakeCmd(tower, TowerEvent.GOTO_L3);
        intakeAndGotoL4 = new JustIntakeCmd(tower, TowerEvent.GOTO_L4);    
        waitToSeeCoral  = new WaitToSeeCoralCmd(tower);

        /* Event Trigger Commands */
        intakeCoral = new TriggerEventCmd(tower, TowerEvent.INTAKE_CORAL);
        score = new TriggerEventCmd(tower, TowerEvent.SCORE);
        intakeLowAlgae = new TriggerEventCmd(tower, TowerEvent.INTAKE_LOW_ALGAE);
        intakeHighAlgae = new TriggerEventCmd(tower, TowerEvent.INTAKE_HIGH_ALGAE);
        gotoL1 = new TriggerEventCmd(tower, TowerEvent.GOTO_L1);
        gotoL3 = new TriggerEventCmd(tower, TowerEvent.GOTO_L3);
        gotoL4 = new TriggerEventCmd(tower, TowerEvent.GOTO_L4);

        /* Waiting Commands */
        waitForAlgae = new WaitForTowerStateCmd(tower, TowerState.WAITING_FOR_ALGAE);
        waitForLowering = new WaitForTowerStateCmd(tower, TowerState.PAUSING_AFTER_SCORING_CORAL);
        waitForHome = new WaitForTowerStateCmd(tower, TowerState.HOME);

        /* Home Elevator Command */
        homeElevator = new HomeElevatorCmd(elevator, tower);

        /* Instant Commands */
        scoreInstant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.SCORE));
        intakeCoralInstant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_CORAL));

        intakeLowAlgaeInstant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_LOW_ALGAE));
        intakeHighAlgaeInstant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_HIGH_ALGAE));

        seedFieldCentricInstant = drive.runOnce(() -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()))).ignoringDisable(true);
        stopDrivetrainInstant = drive.runOnce(drive::stop);
    
        enableSafetyOverrideInstant = vision.runOnce(() -> vision.setSafetyOverride(true));
        //                                                    .andThen(rightVision.runOnce(() -> rightVision.setSafetyOverride(true)));
        enableDirectToAlgaeInstant = tower.runOnce(() -> tower.enableGoToDirectAlgae());

        homeTowerInstant = tower.runOnce(() -> tower.homeTower());
        tiltTowerInstant = tower.runOnce(() -> tower.tiltForward());

        gotoL1Instant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L1));
        gotoL2Instant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L2));
        gotoL3Instant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L3));
        gotoL4Instant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L4));

        startApproachInstant = approach.runOnce(() -> approach.startApproach());
        reefAInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_A));
        reefBInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_B));
        reefABInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_AB));
        reefCInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_C));
        reefDInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_D));
        reefCDInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_CD));
        reefEInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_E));
        reefFInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_F));
        reefEFInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_EF));
        reefGInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_G));
        reefHInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_H));
        reefGHInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_GH));
        reefIInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_I));
        reefJInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_J));
        reefIJInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_IJ));
        reefKInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_K));
        reefLInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_L));
        reefKLInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_KL));
        approachBargeInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.BARGE));
        approachProcessorInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.PROCESSOR));

        /* Robot Centric Movement Commands */
        robotCentricForward = drive.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0.75, 0, 0)));
        robotCentricBackward = drive.runOnce(() -> drive.runVelocity(new ChassisSpeeds(-0.75, 0, 0)));
        robotCentricLeft = drive.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0.25, 0)));
        robotCentricRight = drive.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, -0.25, 0)));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.get();
    }
}
