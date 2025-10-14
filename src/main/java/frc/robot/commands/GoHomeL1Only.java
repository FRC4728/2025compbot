// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Wrist; 
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoHomeL1Only extends SequentialCommandGroup {
  /** Creates a new GoHome. */
  public GoHomeL1Only(Wrist m_Wrist, Elevator m_Elevator) {
    super(
      new SetWristBottom(m_Wrist),
      new WaitCommand(0.5),
      new SetElevatorBottom(m_Elevator));
}
}