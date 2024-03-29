// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndexCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Indexer;

public class IndexShooterToIntake extends Command {
  /** Creates a new IndexConsume. */
  private final Indexer m_Indexer;
  public IndexShooterToIntake(Indexer m_Indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Indexer = m_Indexer;
    addRequirements(m_Indexer);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Indexer.setMotorOutput(ShooterConstants.kMotorConsumeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Indexer.setMotorOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
