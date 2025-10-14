package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlign_Right extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private RobotCentric limeDrive = new RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private double alignmentSpeed, m_xspeed, m_yspeed;
    private int m_pipeline;

    // Constructor accepts limeDrive as a parameter from RobotContainer
    public AutoAlign_Right(CommandSwerveDrivetrain drivetrain, double alignmentSpeed, int pipeline) {
        this.drivetrain = drivetrain;
        this.alignmentSpeed = alignmentSpeed;
        this.m_pipeline = pipeline;
        addRequirements(drivetrain);  // Ensure the drivetrain is required for this command
    }

    @Override
    public void initialize() {
        System.out.println("Caught auto alignment command");
        NetworkTableInstance.getDefault().getTable(Constants.limelightConstants.leftLimelight).getEntry("pipeline").setDouble(m_pipeline);
    }

    @Override
    public void execute() {
        LimelightHelpers.setLEDMode_ForceOn(Constants.limelightConstants.leftLimelight);
        m_xspeed = NetworkTableInstance.getDefault().getTable(Constants.limelightConstants.leftLimelight).getEntry("tx").getDouble(Constants.limelightConstants.defaultValue)*alignmentSpeed;
        m_yspeed = NetworkTableInstance.getDefault().getTable(Constants.limelightConstants.leftLimelight).getEntry("ty").getDouble(Constants.limelightConstants.defaultValue)*alignmentSpeed;
        System.out.println("Running alignment command");
        System.out.println("limelight" +m_xspeed);
        System.out.println("limelight y" +m_yspeed);
        drivetrain.setControl(limeDrive.withVelocityY(-m_xspeed).withVelocityX(-m_yspeed));
        }

    @Override
    public boolean isFinished() {
        // The command finishes when tx is sufficiently close to zero (aligned)
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop robot motion after alignment
}
}