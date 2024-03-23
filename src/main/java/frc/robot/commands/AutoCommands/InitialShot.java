// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InitialShot extends SequentialCommandGroup {
  /** Creates a new BeginShoot. */
  public InitialShot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new WaitCommand(1), 
            NamedCommands.getCommand("ShooterFireFast"),
            NamedCommands.getCommand("SetIntakeGround")),
        new ParallelDeadlineGroup(
          new WaitCommand(0.5), 
            NamedCommands.getCommand("IndexIntakeToShooter")),
        new ParallelDeadlineGroup(
          new WaitCommand(0.1), 
          NamedCommands.getCommand("ShooterStop")
        )
      )
    );
  }
}
