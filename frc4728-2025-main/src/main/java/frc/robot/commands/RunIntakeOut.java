// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunIntakeOut extends Command {
  Intake m_Intake;

  public RunIntakeOut(Intake m_Intake) {
    this.m_Intake = m_Intake;
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  @Override
  public void execute(){
    m_Intake.RunIntake(Constants.IntakeConstants.runIntakeOutVelo);
  }

  @Override
  public void end(boolean interrupted) {
    m_Intake.stop();
  }

  @Override
  public boolean isFinished(){
    return m_Intake.getPhotoCell();
  }
}
