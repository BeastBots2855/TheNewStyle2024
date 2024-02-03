// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeConsume extends Command {
  /** Creates a new IntakeConsume. */
  private final Intake m_Intake;
  private final Supplier<Double> m_SpeedSupplier;
  public IntakeConsume(Intake m_Intake, Supplier<Double> m_SpeedSupplier) {
    this.m_Intake = m_Intake;
    this.m_SpeedSupplier = m_SpeedSupplier;
    addRequirements(m_Intake);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.setMotorOutput(-m_SpeedSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
