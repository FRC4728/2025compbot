// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreL4_HitAlgae extends SequentialCommandGroup {
  /** Creates a new ScoreL4. */
  public ScoreL4_HitAlgae(Wrist m_Wrist, Elevator m_Elevator) {
    super(
      new SetWristIn_L4(m_Wrist),
      new WaitCommand(.45),
      new SetElevatorAlgae2(m_Elevator),
      new HitAlgae(m_Wrist),
      new WaitCommand(0.2),
      new SetElevatorBottom(m_Elevator),
      new WaitCommand(0.3), 
      new SetWristBottom(m_Wrist));
  }
}
