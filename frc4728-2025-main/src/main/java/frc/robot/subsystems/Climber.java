// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  SparkMax m_climberMotor;
  SparkMaxConfig m_motorConfig;
  SparkClosedLoopController m_closedLoopController;
  RelativeEncoder m_encoder;
  DigitalInput limitSwitch;
  boolean m_limitSwitch;

  /** Creates a new Climber. */
  public Climber() {
    m_climberMotor = new SparkMax(Constants.ClimberConstants.climberMotorID, MotorType.kBrushless);

    m_motorConfig = new SparkMaxConfig();

    limitSwitch = new DigitalInput(6);
    m_limitSwitch = false;

    m_closedLoopController = m_climberMotor.getClosedLoopController();
    m_encoder = m_climberMotor.getEncoder();
    
    m_motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(Constants.ClimberConstants.kP)
        .i(Constants.ClimberConstants.kI)
        .d(Constants.ClimberConstants.kD)
        .velocityFF(Constants.ClimberConstants.kFF)
        .outputRange(Constants.minMaxOutputConstants.kMinOutput, Constants.minMaxOutputConstants.kMaxOutput, ClosedLoopSlot.kSlot1);

    m_motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(Constants.ClimberConstants.kMaxMotionVelocity)
        .maxAcceleration(Constants.ClimberConstants.kMaxMotionAcceleration)
        .allowedClosedLoopError(1);
    try{
      m_climberMotor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      System.out.println("!!Successfully configured Climber motor!!");
    }
    catch (Exception e1){
      System.err.println("Failed to apply Climber motor configurations: "+e1.toString());
      DriverStation.reportWarning("Failed to apply Climber motor configuration",true);
    }
    

  }

  @Override
  public void periodic() {
    //periodically get the status of the limit switch
    m_limitSwitch = limitSwitch.get();

    //put the status of the limit switch on the dashboard
    SmartDashboard.putBoolean("Limit Switch Status",m_limitSwitch);

    //put the velocity and position of the climber on the dashboard
    SmartDashboard.putNumber("Climber Velocity",m_encoder.getVelocity());
    SmartDashboard.putNumber("Climber Position",m_encoder.getPosition());
  }

  public void runClimber(double velocity){
    m_closedLoopController.setReference(velocity,ControlType.kVelocity);
  }

  public void resetEncoder(){
    m_encoder.setPosition(0.0);
  }

  public void stop(){
    m_closedLoopController.setReference(0,ControlType.kVelocity);
  }

  public boolean getLimitSwitchClimber(){
    return !m_limitSwitch;
  }
}
