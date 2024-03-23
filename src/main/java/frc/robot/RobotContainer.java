// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.NewShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.subsystems.WristFunctionality.IntakeWrist;
import frc.robot.subsystems.WristFunctionality.ShooterWrist;
import frc.robot.utilities.Autos;
import frc.robot.utilities.ConfigureButtonBindings;
import frc.robot.utilities.PhotonVision;
import frc.robot.utilities.ShuffleBoardInit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private PhotonVision m_PhotonVision = new PhotonVision();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private Intake m_Intake = new Intake();
  private IntakeWrist m_IntakeWrist = new IntakeWrist();
  private ShooterWrist m_ShooterWrist = new ShooterWrist();
  private Shooter m_Shooter = new Shooter();
  private Indexer m_Indexer = new Indexer();
  private Climb m_climb = new Climb();
  private LED m_Led = new LED();
  private NewShooter ShooterTwo = new NewShooter();

  private Autos m_Autos = new Autos(m_robotDrive, m_Shooter);


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    
    new ConfigureButtonBindings(m_driverController, m_operatorController, m_robotDrive, m_Intake, m_Shooter, m_IntakeWrist, m_ShooterWrist, m_Indexer, m_climb, m_Led, m_Autos, ShooterTwo);
    new ShuffleBoardInit(m_driverController, m_operatorController, m_robotDrive, m_Intake, m_Shooter, m_IntakeWrist, m_ShooterWrist, m_Indexer, m_climb);
    

    m_robotDrive.setDefaultCommand(
       new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    m_Shooter.setDefaultCommand(new RunCommand(()-> m_Shooter.setMotorOutput(0.1), m_Shooter));
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
