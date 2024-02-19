// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WristCommands.IntakeWristClosedLoop;
import frc.robot.commands.WristCommands.ShooterWristClosedLoop;
import frc.robot.subsystems.WristFunctionality.IntakeWrist;
import frc.robot.subsystems.WristFunctionality.ShooterWrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIntakeInShooterAmp extends SequentialCommandGroup {
  /** Creates a new MoveToAmp. */
  public SetIntakeInShooterAmp(ShooterWrist m_ShooterWrist, IntakeWrist m_IntakeWrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ShooterWristClosedLoop(m_ShooterWrist, 140).alongWith(new PrintCommand("PIDEnabled"))
        .alongWith(new IntakeWristClosedLoop(m_IntakeWrist, 3)) 
    );
  }
}
