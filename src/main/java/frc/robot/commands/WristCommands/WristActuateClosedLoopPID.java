// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristFunctionality.Wrist;

public class WristActuateClosedLoopPID extends Command {
  /** Creates a new WristActuateClosedLoopPID. */
  private final Wrist m_wrist;
  private final double m_setpoint;

  public WristActuateClosedLoopPID(Wrist m_wrist, double m_setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
      this.m_wrist = m_wrist;
      this.m_setpoint = m_setpoint;
      addRequirements(m_wrist);
      this.m_wrist.enablePid();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_wrist.setSetpoint(m_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.disblePid();
    m_wrist.setMotorOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
