// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.core.base.GeneratorBase;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAlign_Left;
import frc.robot.commands.AutoAlign_Right;
import frc.robot.commands.GoHome;
import frc.robot.commands.GoHomeL1Only;
import frc.robot.commands.GoToL1;
import frc.robot.commands.GoToL2;
import frc.robot.commands.GoToL3;
import frc.robot.commands.GoToL4;
import frc.robot.commands.GrabAlgae;
import frc.robot.commands.RunAlgae1;
import frc.robot.commands.RunAlgae2;
import frc.robot.commands.RunClimberIn;
import frc.robot.commands.RunClimberOut;
import frc.robot.commands.RunIntakeOut;
import frc.robot.commands.ScoreL1;
import frc.robot.commands.ScoreL2_3;
import frc.robot.commands.ScoreL4;
import frc.robot.commands.ScoreL4_HitAlgae;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
    private Elevator elevator =  new Elevator();
    private Intake intake = new Intake();
    private Wrist wrist = new Wrist();
    private Climber climber = new Climber();

    //default values
    public double translationMultiplier = 0.65;
    public double strafeMultiplier = 0.65;
    public double rotateMultipler = 0.8;
    //default deadbands
    public double deadband = 0.08;
    public double rotateDeadband = 0.08;

    //bullshit test boolean
    boolean test;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * deadband).withRotationalDeadband(MaxAngularRate * rotateDeadband) // Add a 8% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandJoystick buttonBoard = new CommandJoystick(1);
    public CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        configureBindings();

        //register commands for path planner
        new EventTrigger("ScoreL2_3").onTrue(new ScoreL2_3(elevator, wrist, intake));
        new EventTrigger("ScoreL1").onTrue(new ScoreL1(elevator, wrist, intake));
        new EventTrigger("ScoreL4").onTrue(new ScoreL4_HitAlgae(wrist, elevator));
        new EventTrigger("GoHome").onTrue(new GoHome(elevator, wrist));
        new EventTrigger("GoHomeL1").onTrue(new GoHomeL1Only(wrist, elevator));
        new EventTrigger("GoToL1").onTrue(new GoToL1(elevator, wrist));
        new EventTrigger("GoToL2").onTrue(new GoToL2(elevator, wrist));
        new EventTrigger("GoToL3").onTrue(new GoToL3(elevator, wrist));
        new EventTrigger("GoToL4").onTrue(new GoToL4(elevator, wrist));
        new EventTrigger("RunAlgae1").onTrue(new RunAlgae1(elevator, wrist, intake));
        new EventTrigger("RunAlgae2").onTrue(new RunAlgae1(elevator, wrist, intake));
        new EventTrigger("RunIntakeOut").onTrue(new RunIntakeOut(intake));

        NamedCommands.registerCommand("AutoAlign_Right",new AutoAlign_Right(drivetrain, 0.05, 0).withTimeout(0.4));
        NamedCommands.registerCommand("AutoAlign_Left",new AutoAlign_Left(drivetrain,0.05,1).withTimeout(0.4));
        NamedCommands.registerCommand("AutoAlign_Right1",new AutoAlign_Right(drivetrain, 0.05, 2).withTimeout(0.4));
        NamedCommands.registerCommand("AutoAlign_Left1",new AutoAlign_Left(drivetrain,0.05,3).withTimeout(0.4));
        NamedCommands.registerCommand("AutoAlign_Right2",new AutoAlign_Right(drivetrain, 0.05, 4).withTimeout(0.4));
        NamedCommands.registerCommand("AutoAlign_Left2",new AutoAlign_Left(drivetrain,0.05,5).withTimeout(0.4));
        NamedCommands.registerCommand("ScoreL4_Command",new ScoreL4_HitAlgae(wrist, elevator).withTimeout(0.6));
        NamedCommands.registerCommand("ScoreL2_3_Command",new ScoreL2_3(elevator, wrist, intake).withTimeout(0.6));
 
        //create auto chooser in dashboard
        autoChooser = AutoBuilder.buildAutoChooser("Main"); 
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(drivetrain.applyRequest(() ->
            drive.withVelocityX(translationMultiplier*-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(strafeMultiplier*-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(rotateMultipler*-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));        
        // reset the field-centric heading on left bumper press
        drivetrain.registerTelemetry(logger::telemeterize);

        //operator controls on button board
        buttonBoard.button(10).whileTrue(new RunClimberOut(climber));
        buttonBoard.button(9).whileTrue(new RunClimberIn(climber));
        buttonBoard.button(1).onTrue(new GoHome(elevator, wrist));
        buttonBoard.button(2).onTrue(new GoToL1(elevator, wrist));
        buttonBoard.button(3).onTrue(new GoToL2(elevator, wrist));
        buttonBoard.button(4).onTrue(new GoToL3(elevator, wrist));
        buttonBoard.button(5).onTrue(new GoToL4(elevator, wrist));
        buttonBoard.button(6).onTrue(new GoHomeL1Only(wrist, elevator));
        buttonBoard.button(7).onTrue(new RunAlgae2(elevator, wrist, intake));
        buttonBoard.button(8).onTrue(new RunAlgae1(elevator, wrist, intake));

        //driver controls
        joystick.y().onTrue(new ScoreL1(elevator, wrist, intake));
        joystick.a().onTrue(new RunIntakeOut(intake));
        joystick.x().onTrue(new ScoreL4_HitAlgae(wrist, elevator));
        joystick.b().onTrue(new ScoreL2_3(elevator, wrist, intake));
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //left bumper to toggle drvetrain to low speed
        joystick.leftBumper().whileTrue(new InstantCommand(() -> translationMultiplier = .13));
        joystick.leftBumper().whileFalse(new InstantCommand(() -> translationMultiplier = 0.65));
        joystick.leftBumper().whileTrue(new InstantCommand(() -> strafeMultiplier = .13));
        joystick.leftBumper().whileFalse(new InstantCommand(() -> strafeMultiplier = 0.65));
        joystick.leftBumper().whileTrue(new InstantCommand(() -> rotateMultipler = .2));
        joystick.leftBumper().whileFalse(new InstantCommand(() -> rotateMultipler = 0.8));

        //slow button deadbands
        joystick.leftBumper().whileTrue(new InstantCommand(() -> deadband = 0.05));
        joystick.leftBumper().whileTrue(new InstantCommand(() -> rotateDeadband = 0.05));

        //normal deadbands
        joystick.leftBumper().whileFalse(new InstantCommand(() -> deadband = 0.08));
        joystick.leftBumper().whileFalse(new InstantCommand(() -> rotateDeadband = 0.08));

        joystick.rightBumper().whileTrue(new AutoAlign_Right(drivetrain, 0.05,0));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected(); 
            
        }

    }

