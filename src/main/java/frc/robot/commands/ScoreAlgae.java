// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAlgae extends SequentialCommandGroup {
  /** Creates a new ScoreL2_3. */
  public ScoreAlgae(Elevator m_Elevator, Wrist m_Wrist, AlgaeManipulator m_algaeManipulator) {
    super(
    new RunAlgaeOut (m_algaeManipulator).withTimeout(0.35),
    new WaitCommand(.05), //was 1
    new SetWristL2_3 (m_Wrist),
    new SetElevatorBottom (m_Elevator),
    new WaitCommand(0.5), 
    new SetWristBottom (m_Wrist));
  }
}

