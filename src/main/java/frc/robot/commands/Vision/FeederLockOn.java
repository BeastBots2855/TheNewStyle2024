// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.utilities.PhotonVision;

public class FeederLockOn extends Command {
  /** Creates a new NoteLockOn. */
  private final DriveSubsystem m_DriveSubsystem;
  private final DoubleSupplier m_XSpeedSupplier;
  private final DoubleSupplier m_YSpeedSupplier;
  private final PIDController m_ThetaController;
  private final Pose2d targetPose;
  public FeederLockOn(DriveSubsystem m_DriveSubsystem, DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_XSpeedSupplier = xSpeedSupplier;
    this.m_YSpeedSupplier = ySpeedSupplier;
    addRequirements(m_DriveSubsystem);
    targetPose = DriverStation.getAlliance().get() ==  DriverStation.Alliance.Red ? 
      FieldConstants.RED_FEEDER_LOCATION :
      FieldConstants.BLUE_FEEDER_LOCATION;


    m_ThetaController = new PIDController(2, 0, 0.001);
    m_ThetaController.enableContinuousInput(-Math.PI, Math.PI);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      //The set point is 0 since that is where the heading of the note is located at
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ThetaController.setSetpoint(PhotonVision.getTagetAngleRobotToTargetPose(targetPose, m_DriveSubsystem.getPose2d()));
    double thetaOutput = m_ThetaController.calculate(m_DriveSubsystem.getPose2d().getRotation().rotateBy(Rotation2d.fromRadians(Math.PI)).getRadians());
    m_DriveSubsystem.drive(
        m_XSpeedSupplier.getAsDouble(),
        m_YSpeedSupplier.getAsDouble(), 
        thetaOutput,
        true,
        false);

        

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
