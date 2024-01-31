// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterEject extends Command {
  /** Creates a new IntakeConsume. */
  private final Shooter m_Shooter;
  private final Supplier<Double> m_SpeedSupplier;
  public ShooterEject(Shooter m_Shooter, Supplier<Double> m_SpeedSupplier) {
    this.m_Shooter = m_Shooter;
    this.m_SpeedSupplier = m_SpeedSupplier;
    addRequirements(m_Shooter);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Shooter.setMotorOutput(m_SpeedSupplier.get());
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