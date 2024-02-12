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
import frc.robot.commands.IndexCommands.IndexConsume;
import frc.robot.commands.IntakeCommands.IntakeConsume;
import frc.robot.commands.IntakeCommands.IntakeEject;
import frc.robot.commands.ShooterCommands.ShooterConsume;
import frc.robot.commands.ShooterCommands.ShooterEject;
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

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  //Joystick m_LogitechController = new Joystick(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    m_telopOutput.addDouble("Box anggle: ", () -> m_ShooterWrist.getAbsoluteEncoderValue());
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
       new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
        
   
        // new RunCommand(
        //   () -> m_robotDrive.drive(
        //       -MathUtil.applyDeadband(m_LogitechController.getRawAxis(1), OIConstants.kDriveDeadband), //left y
        //       -MathUtil.applyDeadband(m_LogitechController.getRawAxis(0), OIConstants.kDriveDeadband), //left x
        //       -MathUtil.applyDeadband(m_LogitechController.getRawAxis(2), OIConstants.kDriveDeadband), // right x
        //       true, true),
        //   m_robotDrive));
  }
//
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its`
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .whileTrue(new InstantCommand(
          m_robotDrive::zeroHeading, 
          m_robotDrive));

    new Trigger(()-> m_operatorController.getLeftTriggerAxis() > 0).whileTrue(
        new IntakeConsume(m_Intake, m_operatorController::getLeftTriggerAxis));

    new Trigger(()-> m_operatorController.getLeftBumper()).whileTrue(
         new IntakeEject(m_Intake));

    new Trigger(()-> m_operatorController.getLeftY() > 0.15 || m_operatorController.getLeftY() < -0.15).whileTrue(
        new WristActuateOpenLoop(m_IntakeWrist, () -> {
            System.out.println(-MathUtil.applyDeadband(m_operatorController.getLeftY() * 0.25, OIConstants.kDriveDeadband));
            return -MathUtil.applyDeadband(m_operatorController.getLeftY(), OIConstants.kDriveDeadband);
        }));

    new Trigger(()-> m_operatorController.getRightBumper()).whileTrue(
         new ShooterConsume(m_Shooter).alongWith(new IndexConsume(m_Indexer)));  
    
    new Trigger(()-> m_operatorController.getRightTriggerAxis() > 0).whileTrue(
        new ShooterEject(m_Shooter, m_operatorController::getRightTriggerAxis)
        .alongWith(new IndexConsume(m_Indexer)));


    new Trigger(()-> m_operatorController.getRightY() > 0.15 || m_operatorController.getRightY() < -0.15).whileTrue(
        new WristActuateOpenLoop(
                m_ShooterWrist, 
                () ->  -MathUtil.applyDeadband(m_operatorController.getRightY(), OIConstants.kDriveDeadband)));

    new Trigger(()-> true).whileTrue(
        new RunCommand(()-> {
            if (m_Intake.shouldRumble()){
                m_operatorController.setRumble(RumbleType.kBothRumble, 1);
                m_driverController.setRumble(RumbleType.kBothRumble, 1);
            } else {
                m_operatorController.setRumble(RumbleType.kBothRumble, 0);
                m_driverController.setRumble(RumbleType.kBothRumble, 0);
            }
        }));

        


    new Trigger(()-> m_operatorController.getAButton()).whileTrue(
         new WristActuateClosedLoopPID(m_ShooterWrist, 30.0).alongWith(new PrintCommand("PIDEnabled"))); 
    // new Trigger(()-> m_operatorController.getBButton()).whileTrue(
    //      new WristActuateClosedLoopPID(m_ShooterWrist, 131.60).alongWith(new PrintCommand("PIDEnabled"))); 
    // new Trigger(()-> m_operatorController.getPOV() == 0).whileTrue(
    //     new WristActuateClosedLoopPID(m_IntakeWrist, 3).alongWith(new PrintCommand("PIDEnabled"))); 
    // new Trigger(()-> m_operatorController.getPOV() == 90).whileTrue(
    //     new WristActuateClosedLoopPID(m_IntakeWrist, 60).alongWith(new PrintCommand("PIDEnabled"))); 
    // new Trigger(()-> m_operatorController.getPOV() == 180).whileTrue(
    //     new WristActuateClosedLoopPID(m_IntakeWrist, 90).alongWith(new PrintCommand("PIDEnabled"))); 
    // new Trigger(()-> m_operatorController.getPOV() == 270).whileTrue(
    //     new WristActuateClosedLoopPID(m_IntakeWrist, 175).alongWith(new PrintCommand("PIDEnabled")));


    
    // new Trigger(()-> m_driverController.getRightTriggerAxis() != 0).whileTrue(new ClimberClimb(m_climb, ()-> m_driverController.getRightTriggerAxis()));
    // new Trigger(()-> m_driverController.getLeftTriggerAxis() != 0).whileTrue(new ClimberClimb(m_climb, ()-> -m_driverController.getLeftTriggerAxis()));

    
      
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

  public void disablePidDisabled(){
    m_IntakeWrist.disblePid();
    m_ShooterWrist.disblePid();
  }


}
