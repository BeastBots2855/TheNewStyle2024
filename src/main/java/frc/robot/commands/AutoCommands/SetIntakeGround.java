// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PIDSetPoint;
import frc.robot.commands.IndexCommands.IndexIntakeToShooter;
import frc.robot.commands.IntakeCommands.IntakeDump;
import frc.robot.commands.WristCommands.IntakeWristClosedLoop;
import frc.robot.commands.WristCommands.ShooterWristClosedLoop;
import frc.robot.subsystems.WristFunctionality.IntakeWrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIntakeGround extends SequentialCommandGroup {
  /** Creates a new SetIntakeGround. */
  public SetIntakeGround(IntakeWrist m_IntakeWrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
        addCommands(
      new ParallelCommandGroup(
            new IntakeWristClosedLoop(m_IntakeWrist, PIDSetPoint.kIntakeGroundPickup)));
  }
}
