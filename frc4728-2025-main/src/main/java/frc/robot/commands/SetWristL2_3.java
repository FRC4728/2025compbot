// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetWristL2_3 extends InstantCommand {
  Wrist m_wrist;

  public SetWristL2_3(Wrist wrist) {
    this.m_wrist = wrist;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  @Override
  public void execute(){
    m_wrist.moveArm(-3.25);
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished(){
    return true;
  }
}
