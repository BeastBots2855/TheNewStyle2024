// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewShooter;

public class NewShooterRescind extends Command {
  /** Creates a new ShooterEject. */

private final NewShooter m_shooter; 
  private final Supplier<Double> m_SpeedSupplier;
  public NewShooterRescind(NewShooter m_shooter, Supplier<Double> m_SpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SpeedSupplier = m_SpeedSupplier;
    this.m_shooter = m_shooter;
    addRequirements(m_shooter);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setMotorRPM(-5000 * m_SpeedSupplier.get());
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_shooter.setMotorVelocities(5000);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotorRPM(500);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
