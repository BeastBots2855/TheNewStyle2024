// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.PhotonVision;

public class NoteLockOn extends Command {
  /** Creates a new NoteLockOn. */
  private final DriveSubsystem m_DriveSubsystem;
  private final DoubleSupplier m_XSpeedSupplier;
  private final DoubleSupplier m_YSpeedSupplier;
  private final PIDController m_ThetaController;
  public NoteLockOn(DriveSubsystem m_DriveSubsystem, DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_XSpeedSupplier = xSpeedSupplier;
    this.m_YSpeedSupplier = ySpeedSupplier;
    addRequirements(m_DriveSubsystem);


    m_ThetaController = new PIDController(0.001, 0, 0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      //The set point is 0 since that is where the heading of the note is located at
      m_ThetaController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thetaOutput = m_ThetaController.calculate(PhotonVision.getNotePidResponseVariable());
    m_DriveSubsystem.drive(
        m_XSpeedSupplier.getAsDouble(),
        m_YSpeedSupplier.getAsDouble(), 
        -thetaOutput,
        false,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_DriveSubsystem.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
