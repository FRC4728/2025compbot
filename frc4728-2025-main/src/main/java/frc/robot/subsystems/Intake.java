package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    TalonFX s_IntakeMotor;
    VelocityVoltage m_velocityVoltage;
    DigitalInput m_photoEye;
    boolean photoCell;

    public Intake() {
        m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

        s_IntakeMotor = new TalonFX(Constants.IntakeConstants.motor1ID);

        m_photoEye = new DigitalInput(Constants.IntakeConstants.photoCellDIO);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = Constants.IntakeConstants.kP;
        config.Slot0.kI = Constants.IntakeConstants.kI;
        config.Slot0.kD = Constants.IntakeConstants.kD;
        config.Slot0.kS = Constants.IntakeConstants.kS;
        config.Slot0.kV = Constants.IntakeConstants.kV;
        config.Slot0.kA = Constants.IntakeConstants.kA;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        try{
            s_IntakeMotor.getConfigurator().apply(config);
            s_IntakeMotor.getConfigurator().apply(config.MotorOutput);
            System.out.println("!!Successfully configured intake motor!!");
        }
        catch(Exception e1){
            DriverStation.reportWarning("Failed to apply intake motor configs",true);
            System.out.println("Failed to apply Intake motor configs: "+e1.toString());
        }

    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake Velocity",s_IntakeMotor.getVelocity().getValueAsDouble());

        //periodically get the status of the photo cell
        photoCell = m_photoEye.get();
        SmartDashboard.putBoolean("Intake Photo Cell Status",photoCell);
    }

    public boolean getPhotoCell(){
        //return the value of the photo cell
        return photoCell;
    }

    public void RunIntake(double Velocity) {
        s_IntakeMotor.setControl(m_velocityVoltage.withVelocity(Velocity));
    }

    public void stop(){
        s_IntakeMotor.setControl(m_velocityVoltage.withVelocity(0));
    }
}
