// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToL1 extends SequentialCommandGroup {
  /** Creates a new GoToL1. */
  public GoToL1(Elevator m_Elevator, Wrist m_Wrist) {
    super(
      new SetWristL1(m_Wrist),
      new SetElevatorL1(m_Elevator));
      
  }
}
