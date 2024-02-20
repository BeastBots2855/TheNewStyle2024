// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.IndexCommands.IndexIntakeToShooter;
import frc.robot.Commands.IntakeCommands.IntakeDump;
import frc.robot.Commands.WristCommands.IntakeWristClosedLoop;
import frc.robot.Commands.WristCommands.ShooterWristClosedLoop;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.WristFunctionality.IntakeWrist;
import frc.robot.Subsystems.WristFunctionality.ShooterWrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundNoteToIndexer extends SequentialCommandGroup {
  /** Creates a new GroundNoteToIndexer. */
  public GroundNoteToIndexer(IntakeWrist m_IntakeWrist, ShooterWrist m_ShooterWrist, Intake m_Intake, Indexer m_Indexer ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
            new IntakeWristClosedLoop(m_IntakeWrist, 180),
            new ShooterWristClosedLoop(m_ShooterWrist, 140))
        .until(()-> m_IntakeWrist.isWithinPidTolerance() && m_ShooterWrist.isWithinPidTolerance())
        .andThen(
            new ParallelCommandGroup(
                new InstantCommand(()-> m_IntakeWrist.setMotorOutput(0), m_IntakeWrist),
                new InstantCommand(()-> m_ShooterWrist.setMotorOutput(0), m_ShooterWrist))
            ).andThen(
            new ParallelDeadlineGroup(
                new WaitCommand(0.38),
                new IntakeDump(m_Intake),
                new IndexIntakeToShooter(m_Indexer))
            ));
  }
}
