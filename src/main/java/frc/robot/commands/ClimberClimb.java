// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb;

public class ClimberClimb extends Command {
  /** Creates a new ClimberClimb. */
  private final Climb m_Climb;
  private final DoubleSupplier m_speedSupplier;
  private final DoubleSupplier m_GryoAngleSupplier;
  public ClimberClimb(Climb m_climb, DoubleSupplier m_speedSupplier, DoubleSupplier m_GryoAngleSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Climb = m_climb;
    this.m_speedSupplier = m_speedSupplier;
    this.m_GryoAngleSupplier = m_GryoAngleSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftOutput = m_speedSupplier.getAsDouble();
    double rightOutput = m_speedSupplier.getAsDouble();
    double angle = m_GryoAngleSupplier.getAsDouble();
    double maxDifferentialAngle = 15;
    double outputDifferential = (1 - (Math.abs(angle /maxDifferentialAngle))); 
    if(angle < 0){
      rightOutput *= outputDifferential;
    } else {
      leftOutput *= outputDifferential;
    }
    m_Climb.setMotorOutput(leftOutput, rightOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Climb.setMotorOutput(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
