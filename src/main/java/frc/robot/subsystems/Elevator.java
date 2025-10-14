package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    TalonFX s_elevator1, s_elevator2;
    NeutralOut s_brake;
    MotionMagicVoltage s_request;


    public Elevator() {
        s_elevator1 = new TalonFX(Constants.ElevatorConstants.topMotorID);
        s_elevator2 = new TalonFX(Constants.ElevatorConstants.bottomMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();

        s_brake = new NeutralOut();

        config.Slot0.kP = Constants.ElevatorConstants.kP;
        config.Slot0.kI = Constants.ElevatorConstants.kI;
        config.Slot0.kD = Constants.ElevatorConstants.kD;
        config.Slot0.kA = Constants.ElevatorConstants.kA;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.ElevatorConstants.kCruiseVelo;
        config.MotionMagic.MotionMagicAcceleration = Constants.ElevatorConstants.kAcceleration;


        // enable stator current limit
        limitConfigs.StatorCurrentLimit = Constants.ElevatorConstants.currentLimit; 
        limitConfigs.StatorCurrentLimitEnable = Constants.ElevatorConstants.enableCurrentLimits;

        //try to apply configuration to motors, throw warning to driver station if it doesn't work
        try{
            s_elevator1.getConfigurator().apply(config.Slot0);
            s_elevator1.getConfigurator().apply(config.MotionMagic);
            s_elevator2.getConfigurator().apply(config.Slot0);
            s_elevator2.getConfigurator().apply(config.MotionMagic);
            s_elevator1.getConfigurator().apply(limitConfigs);
            s_elevator2.getConfigurator().apply(limitConfigs);
            System.out.println("!!Successfully configured elevator motors!!");
        } catch(Exception e1) {
            DriverStation.reportWarning("Failed to apply elevator motor config",true);
            System.out.println("Failed to apply elevator motor config: "+e1.toString());
        }
        //set boolean false to true to invert motor direction
        
        s_elevator1.setPosition(0); //zero encoder?

        s_request = new MotionMagicVoltage(0);
    }

    public void setElevatorSetpoint(double elevatorSetpoint){
        s_elevator1.setControl(s_request.withPosition(elevatorSetpoint));
        s_elevator2.setControl(new Follower(Constants.ElevatorConstants.topMotorID,false));
    }

    @Override
    public void periodic(){

        //periodically output the elevator position and speed
        SmartDashboard.putNumber("Elevator Position:",s_elevator1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Velocity",s_elevator1.getVelocity().getValueAsDouble());
    }

    public void stop(){
        s_elevator2.setControl(s_brake);
    }
}
