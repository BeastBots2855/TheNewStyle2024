// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimberClimb extends Command {
  /** Creates a new ClimberClimb. */
  private final Climb m_Climb;
  private final DoubleSupplier m_speedSupplier;
  public ClimberClimb(Climb m_climb, DoubleSupplier m_speedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Climb = m_climb;
    this.m_speedSupplier = m_speedSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Climb.setMotorOutput(m_speedSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Climb.setMotorOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
