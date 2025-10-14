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
public class ScoreL2_3 extends SequentialCommandGroup {
  /** Creates a new ScoreL2_3. */
  public ScoreL2_3(Elevator m_Elevator, Wrist m_Wrist, Intake m_Intake) {
    super(
    new RunIntakeScore (m_Intake).withTimeout(.5),
    new SetElevatorBottom(m_Elevator),
    new WaitCommand(.25),
    new SetWristBottom(m_Wrist));
  }
}
