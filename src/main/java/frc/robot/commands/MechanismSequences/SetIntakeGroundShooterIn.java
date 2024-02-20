// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.MechanismSequences;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.WristCommands.IntakeWristClosedLoop;
import frc.robot.Commands.WristCommands.ShooterWristClosedLoop;
import frc.robot.Constants.PIDSetPoint;
import frc.robot.subsystems.WristFunctionality.IntakeWrist;
import frc.robot.subsystems.WristFunctionality.ShooterWrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIntakeGroundShooterIn extends SequentialCommandGroup {
  /** Creates a new MoveToAmp. */
  public SetIntakeGroundShooterIn(ShooterWrist m_ShooterWrist, IntakeWrist m_IntakeWrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ShooterWristClosedLoop(m_ShooterWrist, PIDSetPoint.kShooterPassOff).alongWith(new PrintCommand("PIDEnabled"))
        .alongWith(new IntakeWristClosedLoop(m_IntakeWrist, PIDSetPoint.kIntakeGroundPickup)) 
    );
  }
}