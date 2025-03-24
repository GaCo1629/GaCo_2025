// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.HomeElevatorCmd;
import frc.robot.commands.JustIntakeCmd;
import frc.robot.commands.TriggerEventCmd;
import frc.robot.commands.WaitForTowerStateCmd;
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
import frc.robot.Constants.DriverConstants;

public class RobotContainer {
    private final LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    private final AngularVelocity MaxAngularRate = RadiansPerSecond.of(Units.rotationsToRadians(0)); // 3/4 of a rotation per second max angular velocity in radians per second

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed.times(DriverConstants.kDriveDeadband)).withRotationalDeadband(MaxAngularRate.times(DriverConstants.kRotationDeadband)) // Add a 8% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed.in(MetersPerSecond));

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandJoystick       copilot_1 = new CommandJoystick(1);
    private final CommandJoystick       copilot_2 = new CommandJoystick(2);

    static final Transform3d robotToLowerCam = new Transform3d(new Translation3d(0.26, 0.00, 0.20), 
                                                          new Rotation3d(0,0,0));
    static final Vector<N3> lowerCamStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5));


    static final Transform3d robotToUpperCam = new Transform3d(new Translation3d(-0.05, 0.00, 1.017), 
                                                          new Rotation3d(0,Math.toRadians(2.1), Math.PI));
    static final Vector<N3> upperCamStdDevs = VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(10));


    // Instanciate subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final WristSubsystem wrist = new WristSubsystem();
    public final TowerSubsystem tower = new TowerSubsystem(elevator, wrist);
    public final VisionSubsystem lowerVision = new VisionSubsystem(drivetrain, "LowerTagCamera", robotToLowerCam, lowerCamStdDevs, false);
    public final VisionSubsystem upperVision = new VisionSubsystem(drivetrain, "UpperTagCamera", robotToUpperCam, upperCamStdDevs, true);
    public final ApproachSubsystem approach = new ApproachSubsystem(drivetrain);
    
    public final LEDSubsystem led = new LEDSubsystem(0);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        
        // All named commands =========================
        NamedCommands.registerCommand("INTAKE_CORAL",              new TriggerEventCmd(tower, TowerEvent.INTAKE_CORAL));
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L1",        new JustIntakeCmd(tower, TowerEvent.GOTO_L1));
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L2",        new JustIntakeCmd(tower, TowerEvent.GOTO_L2));
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L3",        new JustIntakeCmd(tower, TowerEvent.GOTO_L3));
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L4",        new JustIntakeCmd(tower, TowerEvent.GOTO_L4));
        NamedCommands.registerCommand("SCORE_CORAL",               new TriggerEventCmd(tower, TowerEvent.SCORE));
        NamedCommands.registerCommand("SCORE_ALGAE",               new TriggerEventCmd(tower, TowerEvent.SCORE)); // same as coral
        NamedCommands.registerCommand("GET_ALGAE",                 new TriggerEventCmd(tower, TowerEvent.INTAKE_HIGH_ALGAE));
        NamedCommands.registerCommand("GO_TO_L1",                  new TriggerEventCmd(tower, TowerEvent.GOTO_L1));
        NamedCommands.registerCommand("WAIT_FOR_ALGAE",            new WaitForTowerStateCmd(tower, TowerState.WAITING_FOR_ALGAE));
        NamedCommands.registerCommand("WAIT_FOR_LOWERING",         new WaitForTowerStateCmd(tower, TowerState.PAUSING_AFTER_SCORING_CORAL));
        NamedCommands.registerCommand("WAIT_FOR_HOME",             new WaitForTowerStateCmd(tower, TowerState.HOME));

        NamedCommands.registerCommand("GO_DIRECTLY_TO_ALGAE",      Commands.runOnce(() -> tower.enableGoToDirectAlgae()));
        NamedCommands.registerCommand("DISABLE_HIGH_CAM",          Commands.runOnce(() -> Globals.disableHighCam()));
        NamedCommands.registerCommand("ENABLE_HIGH_CAM",           Commands.runOnce(() -> Globals.enableHighCam()));
        

        // All Path Planner event triggers  ===========
        new EventTrigger("GOTO_L1_ALGAE").onTrue(new TriggerEventCmd(tower, TowerEvent.GOTO_L1));
        new EventTrigger("GOTO_L3_ALGAE").onTrue(new TriggerEventCmd(tower, TowerEvent.GOTO_L3));
        new EventTrigger("INTAKE_LOW_ALGAE").onTrue(new TriggerEventCmd(tower, TowerEvent.INTAKE_LOW_ALGAE));
        new EventTrigger("INTAKE_HIGH_ALGAE").onTrue(new TriggerEventCmd(tower, TowerEvent.INTAKE_HIGH_ALGAE));


        // Configure Auto Chooser  ===============================
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
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
                drive.withVelocityX(MaxSpeed.in(MetersPerSecond) * -joystick.getLeftY() * Constants.DriverConstants.kMaxDriveSpeedPercentage.in(Percent) * tower.getTowerSpeedSafetyFactor().in(Percent)) // Drive forward with negative Y (forward)
                    .withVelocityY(MaxSpeed.in(MetersPerSecond) * -joystick.getLeftX() * Constants.DriverConstants.kMaxDriveSpeedPercentage.in(Percent) * tower.getTowerSpeedSafetyFactor().in(Percent)) // Drive left with negative X (left)
                    .withRotationalRate(MaxAngularRate.in(RadiansPerSecond) * -joystick.getRightX() * Constants.DriverConstants.kMaxTurnSpeedPercentage.in(Percent) * tower.getTowerSpeedSafetyFactor().in(Percent)) // Drive counterclockwise with negative X (left)
            ));

        // avoid the PathPlanner startup delay....
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {

        // Driver Buttons

        joystick.rightTrigger(0.5).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.SCORE)));  // score coral or algae

        joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));  // reset field centric home


        joystick.start().onTrue(new HomeElevatorCmd(elevator, tower));  //home the elevator

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> runCoralStationCmd(true)));  // collect coral left side
        joystick.rightBumper().onTrue(drivetrain.runOnce(() -> runCoralStationCmd(false)));// collect coral right side

        joystick.y().onTrue(drivetrain.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_HIGH_ALGAE)));
        joystick.a().onTrue(drivetrain.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_LOW_ALGAE)));

        joystick.x().onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.BARGE)));
        joystick.b().onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.PROCESSOR)));

        // ==== Approach Buttons ================================

        joystick.leftTrigger(0.5).onTrue(approach.runOnce(() -> approach.startApproach()))
        .onFalse(drivetrain.runOnce(() -> drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.0).withVelocityY(0.0))));

        // ==== NON Field Centric driving ================================

        LinearVelocity xSpeed = MetersPerSecond.of(0.75);
        LinearVelocity ySpeed = MetersPerSecond.of(0.25);

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(xSpeed).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(xSpeed.in(MetersPerSecond) * -1.0).withVelocityY(0))
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(ySpeed.in(MetersPerSecond) * -1.0))
        );
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(ySpeed))
        );
            
        // ====  CoPilot 1 Buttons  ======================================

        copilot_1.button(DriverConstants.reset).onTrue(lowerVision.runOnce(() -> lowerVision.setSafetyOverride(true))
//                                             .andThen(upperVision.runOnce(() -> upperVision.setSafetyOverride(true)))  //   only override low cam safety to reposition
                                               );

        copilot_1.button(DriverConstants.home).onTrue(tower.runOnce(() -> tower.homeTower()));

        copilot_1.button(DriverConstants.l1).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L1)));
        copilot_1.button(DriverConstants.l2).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L2)));
        copilot_1.button(DriverConstants.l3).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L3)));
        copilot_1.button(DriverConstants.l4).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L4)));

        copilot_1.button(DriverConstants.pose_i).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_I)));
        copilot_1.button(DriverConstants.pose_j).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_J)));
        copilot_1.button(DriverConstants.pose_ija).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_IJ)));
        copilot_1.button(DriverConstants.pose_k).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_K)));
        copilot_1.button(DriverConstants.pose_l).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_L)));
        copilot_1.button(DriverConstants.pose_kla).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_KL)));
        
        // ===  CoPilot 2 Buttons  ===========================================

        copilot_2.button(DriverConstants.reset).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.HOME_TOWER)));
        copilot_2.button(DriverConstants.pose_a).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_A)));
        copilot_2.button(DriverConstants.pose_b).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_B)));
        copilot_2.button(DriverConstants.pose_aba).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_AB)));
        copilot_2.button(DriverConstants.pose_c).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_C)));
        copilot_2.button(DriverConstants.pose_d).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_D)));
        copilot_2.button(DriverConstants.pose_cda).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_CD)));
        copilot_2.button(DriverConstants.pose_e).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_E)));
        copilot_2.button(DriverConstants.pose_f).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_F)));
        copilot_2.button(DriverConstants.pose_efa).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_EF)));
        copilot_2.button(DriverConstants.pose_g).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_G)));
        copilot_2.button(DriverConstants.pose_h).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_H)));
        copilot_2.button(DriverConstants.pose_gha).onTrue(tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_GH)));
       
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    // ==============================================================================================
    // Approach Command code
    // ==============================================================================================
    private Command approachCoralStationCommand;
    boolean isAtTarget = false;
    MutAngle targetAngle = Degrees.mutable(0.0); // Initial target angle
    MutAngle headingError = Degrees.mutable(0.0);
    MutAngle robotRotation = Degrees.mutable(drivetrain.getState().Pose.getRotation().getDegrees());

    BooleanSupplier atTarget = (() -> {
        robotRotation.mut_replace(drivetrain.getState().Pose.getRotation().getDegrees(), Degrees);
        
        // targetAngle - drivetrainRotation > 180
        if(targetAngle.in(Degrees) - robotRotation.in(Degrees) > 180.0) {
            // targetAnngle - drivetrainRotation - 360
            headingError.mut_replace(targetAngle.in(Degrees) -  robotRotation.in(Degrees) - 360.0, Degrees);
        } 
        // targetAngle - drivetrainRotation < -180
        else if(targetAngle.in(Degrees) - robotRotation.in(Degrees) > 180.0) {
            // targetAnngle - drivetrainRotation + 360
            headingError.mut_replace(targetAngle.in(Degrees) - robotRotation.in(Degrees) + 360.0, Degrees);
        } 
        // -180 < targetAngle - drivetrainRotation < 180
        else{
            // targetAnngle - drivetrainRotation
            headingError.mut_replace(targetAngle.in(Degrees) - robotRotation.in(Degrees), Degrees);
        }

        // |targetAngle - drivetrainRotation| < 1.0
        if(Math.abs(targetAngle.in(Degrees) - robotRotation.in(Degrees)) < 1.0){
            return true;
        } else{
            SmartDashboard.putNumber("Degrees Left to Turn", headingError.in(Degrees));
            return false;
        }
    });

    public void faceCoralStation(boolean isLeft){
        // Choose correct tag based on alliance color and side
        int tagId = 13;
        if(!isLeft){
            tagId -= 1;
        }
        approach.alliance = DriverStation.getAlliance();
        if(approach.alliance.get().equals(Alliance.Red)){
            tagId -= 11;
            if(isLeft){
                tagId -= 1;
            } else{
                tagId += 1;
            }
        }
        targetAngle.mut_replace(approach.tags.getTagPose(tagId).get().getRotation().toRotation2d().getDegrees(),  Degrees);

        // targetAngle - drivetrainRotation > 180
        if(targetAngle.in(Degrees) - robotRotation.in(Degrees) > 180.0){
            // targetAngle - drivetrainRoation + 360
            headingError.mut_replace(targetAngle.in(Degrees) - robotRotation.in(Degrees) + 360.0, Degrees);
        } else{
            // targetAngle - drivetrainRotation
            headingError.mut_replace(targetAngle.in(Degrees) - robotRotation.in(Degrees), Degrees);
        }
            approachCoralStationCommand = drivetrain.applyRequest(() ->
                drive.withVelocityX(MaxSpeed.in(MetersPerSecond) * -joystick.getLeftY() / 2.0) // Drive forward with negative Y (forward)
                    .withVelocityY(MaxSpeed.in(MetersPerSecond) * -joystick.getLeftX() / 2.0) // Drive left with negative X (left)
                    // MaxAngularrate * headingError / 80 = angularVelocity
                    .withRotationalRate(clampAngularVelocity(MaxAngularRate.in(RadiansPerSecond) * headingError.in(Radians) / 80.0, -Math.PI, Math.PI, Math.PI / 4.0, RadiansPerSecond)) // Auto rotate to position
            ).until(atTarget); // Run until target angle is reached
      }


    private CommandScheduler scheduler = CommandScheduler.getInstance();

    public void runCoralStationCmd(boolean isLeft) {
        faceCoralStation(isLeft);
        scheduler.schedule(approachCoralStationCommand);
        tower.triggerEvent(TowerEvent.INTAKE_CORAL);
    }

    private AngularVelocity clampAngularVelocity(double value, double min, double max, double innerBound, AngularVelocityUnit unit){
        if(value > max){
            value = max;
        } else if(value > 0.0 && value < innerBound){
            value = innerBound;
        } else if(value < min){
            value = min;
        } else if(value > 0.0 && value > (innerBound * -1.0)) {
            value = innerBound * -1.0;
        }
        return unit.of(value);
    }
}
