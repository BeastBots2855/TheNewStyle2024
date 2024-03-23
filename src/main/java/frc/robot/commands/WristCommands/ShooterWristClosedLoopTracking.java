// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristFunctionality.ShooterWrist;
import frc.robot.subsystems.WristFunctionality.Wrist;

public class ShooterWristClosedLoopTracking extends Command {
  /** Creates a new WristActuateClosedLoopPID. */
  private final Wrist m_wrist;
  private final DoubleSupplier m_setpoint;

  public ShooterWristClosedLoopTracking(Wrist m_wrist, DoubleSupplier m_setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
      this.m_wrist = m_wrist;
      this.m_setpoint = m_setpoint;
      addRequirements((ShooterWrist)m_wrist);
      
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      this.m_wrist.enablePid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
          m_wrist.setSetpoint(m_setpoint.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
