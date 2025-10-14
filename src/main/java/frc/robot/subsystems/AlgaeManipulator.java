package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeManipulator extends SubsystemBase{
    SparkMax m_algaeMotor;
    SparkMaxConfig m_algaeMotorConfig;
    SparkClosedLoopController m_algaeClosedLoopController;
    RelativeEncoder m_relativeEncoder;

    public AlgaeManipulator() {
        m_algaeMotor = new SparkMax(Constants.IntakeConstants.Algae.algaeMotorID, MotorType.kBrushless);

        m_algaeClosedLoopController = m_algaeMotor.getClosedLoopController();

        m_algaeMotorConfig = new SparkMaxConfig();

        m_relativeEncoder = m_algaeMotor.getEncoder();

        m_algaeMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Constants.IntakeConstants.Algae.kP)
            .i(Constants.IntakeConstants.Algae.kI)
            .d(Constants.IntakeConstants.Algae.kD)
            .velocityFF(Constants.IntakeConstants.Algae.kFF)
            .outputRange(-1,1);
            
        m_algaeMotorConfig.smartCurrentLimit(40);

        try{
            m_algaeMotor.configure(m_algaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            System.out.println("!!Successfully configured algae motor!!");
        }
        catch (Exception e2){
            System.err.println("Failed to apply algae motor configurations: "+e2.toString());
            DriverStation.reportWarning("Failed to apply algae motor configuration",true);
        }

    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Algae Manipulator Velocity",m_relativeEncoder.getVelocity());
    }

    public void run(double velocity){
        m_algaeClosedLoopController.setReference(velocity,ControlType.kVelocity);
    }
    
    public void stop(){
        m_algaeClosedLoopController.setReference(0,ControlType.kVelocity);
    }
}