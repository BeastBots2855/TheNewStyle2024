// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimitSwitchConstants;
import frc.robot.subsystems.Wrist;

public class WristActuateOpenLoop extends Command {
  /** Creates a new WristActuate. */
  private final Wrist m_wrist;
  private final Supplier<Double> m_SpeedSupplier;
  public WristActuateOpenLoop(Wrist m_wrist, Supplier<Double> m_SpeedSupplier) {
    this.m_wrist = m_wrist;
    this.m_SpeedSupplier = m_SpeedSupplier;
    addRequirements(m_wrist);
    this.m_wrist.disable();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setMotorOutput(m_SpeedSupplier.get() * 0.25);
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
