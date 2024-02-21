// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands.IntakeConsume;
import frc.robot.commands.MechanismSequences.GroundNoteToIndexer;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.WristFunctionality.IntakeWrist;
import frc.robot.subsystems.WristFunctionality.ShooterWrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntake extends SequentialCommandGroup {
  /** Creates a new IntakeUntillIn. */
  public AutoIntake(IntakeWrist m_IntakeWrist, ShooterWrist m_ShooterWrist, Intake m_Intake, Indexer m_Indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeConsume(m_Intake, ()-> 0.5).until(()->m_Intake.isTouchingLimitSwitch()).andThen(new GroundNoteToIndexer(m_IntakeWrist, m_ShooterWrist, m_Intake, m_Indexer)));
  }
}
