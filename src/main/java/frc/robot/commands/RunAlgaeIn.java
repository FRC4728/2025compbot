// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeManipulator;
//import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAlgaeIn extends Command {
  AlgaeManipulator m_algaeManipulator;

  public RunAlgaeIn(AlgaeManipulator m_algaeManipulator) {
    this.m_algaeManipulator = m_algaeManipulator;
    addRequirements(m_algaeManipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  @Override
  public void execute(){
    m_algaeManipulator.run(Constants.IntakeConstants.Algae.inVelo);
  }

  @Override
  public void end(boolean interrupted) {
    m_algaeManipulator.stop();
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}