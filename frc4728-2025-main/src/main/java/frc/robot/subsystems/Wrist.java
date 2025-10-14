package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase{
    TalonFX m_wristMotor;
    NeutralOut m_brake;
    MotionMagicVoltage m_request;
    CANcoder m_wristEncoder;

    public Wrist() {
        m_wristMotor = new TalonFX(Constants.WristConstants.motor1ID);

        m_wristEncoder = new CANcoder(Constants.WristConstants.cancoderID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        m_brake = new NeutralOut();

        config.Slot0.kP = Constants.WristConstants.kP;
        config.Slot0.kI = Constants.WristConstants.kI;
        config.Slot0.kD = Constants.WristConstants.kD;
        config.Slot0.kA = Constants.WristConstants.kA;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        //config.Feedback.FeedbackRemoteSensorID = m_wristEncoder.getDeviceID();
        //config.Feedback.RotorToSensorRatio=12.8;
        //config.Feedback.SensorToMechanismRatio=1.0;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.WristConstants.kCruiseVelo;
        config.MotionMagic.MotionMagicAcceleration = Constants.WristConstants.kAcceleration;


        // enable stator current limit
        limitConfigs.StatorCurrentLimit = Constants.WristConstants.currentLimit; 
        limitConfigs.StatorCurrentLimitEnable = Constants.WristConstants.enableCurrentLimits;

        //try to apply configurations to motors, throw warning to driver station if it doesn't work
        try{
            m_wristMotor.getConfigurator().apply(config.Slot0);
            m_wristMotor.getConfigurator().apply(config.MotionMagic);
            m_wristMotor.getConfigurator().apply(config.MotorOutput);
            m_wristMotor.getConfigurator().apply(limitConfigs);
            m_wristMotor.getConfigurator().apply(config.Feedback);
            System.out.println("!!Successfully configured Wrist motor!!");

        } catch(Exception e1) {
            DriverStation.reportWarning("Failed to apply wrist motor configs",true);
            System.out.println("Failed to apply Wrist motor configs: "+e1.toString());
        }
        //set boolean false to true to invert motor direction

        m_request = new MotionMagicVoltage(0);

        m_wristMotor.setPosition(0);

        m_wristEncoder.setPosition(0);

    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Wrist Motor Position",m_wristMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Wrist CANcoder Position",m_wristEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Wrist Velocity",m_wristMotor.getVelocity().getValueAsDouble());
    }

    public void moveArm(double setpoint){
        m_wristMotor.setControl(m_request.withPosition(setpoint));
    }

    public void stop(){
        m_wristMotor.setControl(m_brake);
    }
}
