// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristFunctionality.IntakeWrist;
import frc.robot.subsystems.WristFunctionality.Wrist;

public class ShooterWristOpenLoop extends Command {
  /** Creates a new WristActuate. */
  private final Wrist m_wrist;
  private final Supplier<Double> m_SpeedSupplier;
  public ShooterWristOpenLoop(Wrist m_wrist, Supplier<Double> m_SpeedSupplier) {
    this.m_wrist = m_wrist;
    this.m_SpeedSupplier = m_SpeedSupplier;
    addRequirements((IntakeWrist)m_wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      this.m_wrist.disblePid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setMotorOutput(m_SpeedSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setMotorOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
