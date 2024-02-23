// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LedCommands;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj2.command.Command;

public class RAINBOWS extends Command {
  private final LED m_ledString;
  int counter = 0;
  /** Creates a new RAINBOWS. */
  public RAINBOWS(LED subsystem) {
    m_ledString = subsystem;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_ledString.setRainbow();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   m_ledString.setRainbow();
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
