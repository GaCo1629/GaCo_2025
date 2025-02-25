// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.SetElevatorHeight;
import frc.robot.commands.SetFinAngle;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TowerEvent;
import frc.robot.subsystems.TowerSubsystem;;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // Instanciate subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final WristSubsystem wrist = new WristSubsystem();
    public final TowerSubsystem tower = new TowerSubsystem(elevator, wrist);
    public final VisionSubsystem vision = new VisionSubsystem(drivetrain);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        
        NamedCommands.registerCommand("INTAKE_CORAL", tower.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_CORAL)));
        NamedCommands.registerCommand("GOTO_L1", tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L1)));
        NamedCommands.registerCommand("GOTO_L2", tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L2)));
        NamedCommands.registerCommand("GOTO_L3", tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L3)));
        NamedCommands.registerCommand("GOTO_L4", tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L4)));
        NamedCommands.registerCommand("SCORE_CORAL", tower.runOnce(() -> tower.triggerEvent(TowerEvent.SCORE_CORAL)));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed / 5) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed / 5) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate / 5) // Drive counterclockwise with negative X (left)
            )
        );
    }

    private void configureBindings() {


        // Tower State Machine Events
        joystick.leftBumper().onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_CORAL)));

        joystick.pov(180).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L1)));
        joystick.pov(270).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L2)));
        joystick.pov(90).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L3)));
        joystick.pov(0).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L4)));

        joystick.rightBumper().onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.SCORE_CORAL)));

        // reset the field-centric heading on left bumper press
        joystick.back().and(joystick.start()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
         
        /*
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );

        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.5))
        );
        */

        /* 
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
