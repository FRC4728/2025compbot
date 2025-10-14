// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootL4 extends SequentialCommandGroup {
  /** Creates a new ScoreL4. */
  public ShootL4(Wrist m_Wrist, Elevator m_Elevator, Intake m_Intake) {
    addCommands(
      new SetWristShootL4(m_Wrist),
      new WaitCommand(.3),
      new RunIntakeScore(m_Intake).withTimeout(.4),
      new SetWristL4(m_Wrist),
      new WaitCommand(0.6),
      new SetElevatorBottom(m_Elevator),
      new WaitCommand(0.3), 
      new SetWristBottom(m_Wrist));
  }
}
