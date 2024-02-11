// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeWristConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterWristConstants;
import frc.robot.commands.ClimberClimb;
import frc.robot.commands.IndexCommands.IndexIntakeToShooter;
import frc.robot.commands.IntakeCommands.IntakeConsume;
import frc.robot.commands.IntakeCommands.IntakeDump;
import frc.robot.commands.ShooterCommands.ShooterRecieve;
import frc.robot.commands.ShooterCommands.ShooterFire;
import frc.robot.commands.WristCommands.WristActuateClosedLoopPID;
import frc.robot.commands.WristCommands.WristActuateOpenLoop;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WristFunctionality.IntakeWrist;
import frc.robot.subsystems.WristFunctionality.ShooterWrist;
import frc.robot.subsystems.WristFunctionality.Wrist;
import frc.robot.utilities.Autos;
import frc.robot.utilities.ConfigureButtonBindings;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private ShuffleboardTab m_telopOutput = Shuffleboard.getTab("Teleop");
  private Intake m_Intake = new Intake();
  private IntakeWrist m_IntakeWrist = new IntakeWrist(0.01, 0.001, 0, 0.03, IntakeWristConstants.IntakeWristCANID);
  private ShooterWrist m_ShooterWrist = new ShooterWrist(0.05,0,0,0, ShooterWristConstants.ShooterWristCANID);
  private Shooter m_Shooter = new Shooter();
  private Indexer m_Indexer = new Indexer();
  private Climb m_climb = new Climb();

  private Autos m_Autos = new Autos(m_robotDrive, m_Shooter);


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    m_telopOutput.addDouble("Box anggle: ", () -> m_ShooterWrist.getAbsoluteEncoderValue());
    new ConfigureButtonBindings(m_driverController, m_operatorController, m_robotDrive, m_Intake,
        m_Shooter, m_IntakeWrist, m_ShooterWrist, m_Indexer, m_climb);
    

    m_robotDrive.setDefaultCommand(
       new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }
//
 
      
  



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_Autos.getAutoCommand();
  }

  public void disablePidDisabled(){
    m_IntakeWrist.disblePid();
    m_ShooterWrist.disblePid();
  }


}
