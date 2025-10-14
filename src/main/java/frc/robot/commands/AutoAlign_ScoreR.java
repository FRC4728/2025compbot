// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAlign_ScoreR extends SequentialCommandGroup {
  CommandSwerveDrivetrain drivetrain;
  Wrist wrist;
  Elevator elevator;
  int pipeline;
  /** Creates a new AutoAlign_Left1. */
  public AutoAlign_ScoreR(CommandSwerveDrivetrain drivetrain, Wrist wrist, Elevator elevator, int pipeline) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(0.5),
      new AutoAlign_Right(drivetrain, 0.05, pipeline).withTimeout(0.5),
      new ScoreL4_HitAlgae(wrist, elevator),
      new WaitCommand(0.5)
    );
  }
}
