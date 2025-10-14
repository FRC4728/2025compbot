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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAlign_Left;
import frc.robot.commands.AutoAlign_Right;
import frc.robot.commands.AutoAlign_ScoreR;
import frc.robot.commands.GoHome;
import frc.robot.commands.GoHomeL1Only;
import frc.robot.commands.GoToBarge;
import frc.robot.commands.GoToL1;
import frc.robot.commands.GoToL2;
import frc.robot.commands.GoToL3;
import frc.robot.commands.GoToL4;
import frc.robot.commands.GoToLolipop;
import frc.robot.commands.GoToNewL1;
import frc.robot.commands.GoToProcessor;
import frc.robot.commands.GoToReef1;
import frc.robot.commands.GoToReef2;
import frc.robot.commands.GrabAlgae;
import frc.robot.commands.IntakeGroundAlgae;
import frc.robot.commands.RunAlgae1;
import frc.robot.commands.RunAlgae2;
import frc.robot.commands.RunAlgaeIdle;
import frc.robot.commands.RunAlgaeIn;
import frc.robot.commands.RunClimberIn;
import frc.robot.commands.RunClimberOut;
import frc.robot.commands.RunIntakeOut;
import frc.robot.commands.ScoreAlgae;
import frc.robot.commands.ScoreL1;
import frc.robot.commands.ScoreL2_3;
import frc.robot.commands.ScoreL4;
import frc.robot.commands.ScoreL4_Auto;
import frc.robot.commands.ScoreL4_HitAlgae;
import frc.robot.commands.ScoreL4_MiddleAuto;
import frc.robot.commands.ScoreNewL1;
import frc.robot.commands.ShootL4;
import frc.robot.commands.RunAlgaeOut;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeManipulator;
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
    private AlgaeManipulator algaeManipulator = new AlgaeManipulator();

    //default values
    public double translationMultiplier = 0.85;
    public double strafeMultiplier = 0.85;
    public double rotateMultipler = 0.75;
    //default deadbands
    public double deadband = 0.08;
    public double rotateDeadband = 0.08;

    //bullshit test boolean
    boolean test;
    DigitalInput m_photoEye;
    boolean photoCell;

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
    public void periodic(){

        //periodically get the status of the photo cell
        photoCell = m_photoEye.get();
        SmartDashboard.putBoolean("Intake Photo Cell Status",photoCell);
    }

    public boolean getPhotoCell(){
        //return the value of the photo cell
        return photoCell;
    }
    public RobotContainer() {
        //register commands for path planner
        new EventTrigger("ScoreL2_3").onTrue(new ScoreL2_3(elevator, wrist, intake));
        new EventTrigger("ScoreL1").onTrue(new ScoreL1(elevator, wrist, intake));
        new EventTrigger("ScoreL4").onTrue(new ScoreL4_HitAlgae(wrist, elevator));
        new EventTrigger("ScoreL4_Old").onTrue(new ScoreL4(wrist, elevator));
        new EventTrigger("GoHome").onTrue(new GoHome(elevator, wrist));
        new EventTrigger("GoHomeL1").onTrue(new GoHomeL1Only(wrist, elevator));
        new EventTrigger("GoToL1").onTrue(new GoToNewL1(elevator, wrist));
        new EventTrigger("GoToL2").onTrue(new GoToL2(elevator, wrist));
        new EventTrigger("GoToL3").onTrue(new GoToL3(elevator, wrist));
        new EventTrigger("GoToL4").onTrue(new GoToL4(elevator, wrist));
        new EventTrigger("RunAlgae1").onTrue(new RunAlgae1(elevator, wrist, intake));
        new EventTrigger("RunAlgae2").onTrue(new RunAlgae1(elevator, wrist, intake));
        new EventTrigger("RunIntakeOut").onTrue(new RunIntakeOut(intake));
        new EventTrigger("GoToAlgae1").onTrue(new GoToReef1(elevator, wrist, algaeManipulator));
        new EventTrigger("RunAlgaeIdle").onTrue(new RunAlgaeIdle(algaeManipulator));
        new EventTrigger("ShootAlgae").onTrue(new RunAlgaeOut(algaeManipulator));
        new EventTrigger("GoToAlgae2").onTrue(new GoToReef2(elevator, wrist, algaeManipulator));
        new EventTrigger("ScoreL4_Middle").onTrue(new ScoreL4_MiddleAuto(wrist, elevator, algaeManipulator));

        NamedCommands.registerCommand("AutoAlign_BD_Right",new AutoAlign_Right(drivetrain, 0.04, 0).withTimeout(0.5));
        NamedCommands.registerCommand("AutoAlign_BH_Right",new AutoAlign_Right(drivetrain, 0.04, 1).withTimeout(0.5));
        NamedCommands.registerCommand("AutoAlign_BJ_Right",new AutoAlign_Right(drivetrain, 0.04, 2).withTimeout(0.5));
        NamedCommands.registerCommand("AutoAlign_BL_Right",new AutoAlign_Right(drivetrain, 0.04, 3).withTimeout(0.5));

        NamedCommands.registerCommand("AutoAlign_RD_Right",new AutoAlign_Right(drivetrain, 0.04, 4).withTimeout(0.5));
        NamedCommands.registerCommand("AutoAlign_RH_Right",new AutoAlign_Right(drivetrain, 0.04, 5).withTimeout(0.5));
        NamedCommands.registerCommand("AutoAlign_RJ_Right",new AutoAlign_Right(drivetrain, 0.04, 6).withTimeout(0.5));
        NamedCommands.registerCommand("AutoAlign_RL_Right",new AutoAlign_Right(drivetrain, 0.04, 7).withTimeout(0.5));

        NamedCommands.registerCommand("AutoAlign_BC_Left",new AutoAlign_Left(drivetrain, 0.04, 0).withTimeout(0.5));
        NamedCommands.registerCommand("AutoAlign_BE_Left",new AutoAlign_Left(drivetrain, 0.04, 1).withTimeout(0.5));
        NamedCommands.registerCommand("AutoAlign_BG_Left",new AutoAlign_Left(drivetrain, 0.04, 2).withTimeout(0.5));
        NamedCommands.registerCommand("AutoAlign_BK_Left",new AutoAlign_Left(drivetrain, 0.04, 3).withTimeout(0.5));

        NamedCommands.registerCommand("AutoAlign_RC_Left",new AutoAlign_Left(drivetrain, 0.04, 4).withTimeout(0.5));
        NamedCommands.registerCommand("AutoAlign_RE_Left",new AutoAlign_Left(drivetrain, 0.04, 5).withTimeout(0.5));
        NamedCommands.registerCommand("AutoAlign_RG_Left",new AutoAlign_Left(drivetrain, 0.04, 6).withTimeout(0.5));
        NamedCommands.registerCommand("AutoAlign_RK_Left",new AutoAlign_Left(drivetrain, 0.04, 7).withTimeout(0.5));

        NamedCommands.registerCommand("ScoreL4_Command",new ScoreL4_HitAlgae(wrist, elevator).withTimeout(1));
        NamedCommands.registerCommand("ScoreL2_3_Command",new ScoreL2_3(elevator, wrist, intake));
 
        //create auto chooser in dashboard
        autoChooser = AutoBuilder.buildAutoChooser("Main"); 
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        configureBindings();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(drivetrain.applyRequest(() ->
            drive.withVelocityX(translationMultiplier*-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(strafeMultiplier*-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(rotateMultipler*-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));        
        // reset the field-centric heading on left bumper press
        drivetrain.registerTelemetry(logger::telemeterize);

        algaeManipulator.setDefaultCommand(new RunAlgaeIdle(algaeManipulator));

        //algaeManipulator.setDefaultCommand(new RunAlgaeIdle(algaeManipulator));

        //operator controls on button board
        buttonBoard.button(1).onTrue(new GoHome(elevator, wrist));
        buttonBoard.button(2).onTrue(new GoToNewL1(elevator, wrist));
        buttonBoard.button(3).onTrue(new GoToL2(elevator, wrist));
        buttonBoard.button(4).onTrue(new GoToL3(elevator, wrist));
        buttonBoard.button(5).onTrue(new GoToL4(elevator, wrist));
        buttonBoard.button(6).onTrue(new IntakeGroundAlgae(elevator, wrist, algaeManipulator));
        buttonBoard.button(8).onTrue(new GoToReef1(elevator, wrist, algaeManipulator));
        buttonBoard.button(7).onTrue(new GoToReef2(elevator, wrist, algaeManipulator));
        buttonBoard.button(9).whileTrue(new RunClimberIn(climber));
        buttonBoard.button(10).whileTrue(new RunClimberOut(climber));
        buttonBoard.button(11).onTrue(new GoToProcessor(elevator, wrist));
        //buttonBoard.button(12).whileFalse(new RunAlgaeIdle(algaeManipulator));
        buttonBoard.button(12).onTrue(new GoToLolipop(elevator, wrist, algaeManipulator));
        //buttonBoard.button(12).onTrue(new AutoAlign_Left(drivetrain, 0.04, 0));

        //driver controls
        joystick.y().onTrue(new ScoreNewL1(elevator, wrist, intake));
        joystick.a().onTrue(new RunIntakeOut(intake));
        joystick.x().onTrue(new ScoreL4(wrist, elevator));
        joystick.b().onTrue(new ScoreL2_3(elevator, wrist, intake));
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); 
        joystick.rightBumper().onTrue(new ScoreAlgae(elevator, wrist, algaeManipulator));

        //left bumper to toggle drvetrain to low speed
        joystick.leftBumper().whileTrue(new InstantCommand(() -> translationMultiplier = .13));
        joystick.leftBumper().whileFalse(new InstantCommand(() -> translationMultiplier = 0.85));
        joystick.leftBumper().whileTrue(new InstantCommand(() -> strafeMultiplier = .13));
        joystick.leftBumper().whileFalse(new InstantCommand(() -> strafeMultiplier = 0.85));
        joystick.leftBumper().whileTrue(new InstantCommand(() -> rotateMultipler = .2));
        joystick.leftBumper().whileFalse(new InstantCommand(() -> rotateMultipler = 0.75));

        //slow button deadbands
        joystick.leftBumper().whileTrue(new InstantCommand(() -> deadband = 0.05));
        joystick.leftBumper().whileTrue(new InstantCommand(() -> rotateDeadband = 0.05));

        //normal deadbands
        joystick.leftBumper().whileFalse(new InstantCommand(() -> deadband = 0.08));
        joystick.leftBumper().whileFalse(new InstantCommand(() -> rotateDeadband = 0.08));

    }
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected(); 
             
        }
}
