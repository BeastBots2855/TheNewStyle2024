// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IndexCommands.IndexIntakeToShooter;
import frc.robot.commands.IntakeCommands.IntakeConsume;
import frc.robot.commands.IntakeCommands.IntakeDump;
import frc.robot.commands.ShooterCommands.ShooterFire;
import frc.robot.commands.ShooterCommands.ShooterRecieve;
import frc.robot.commands.WristCommands.WristActuateClosedLoopPID;
import frc.robot.commands.WristCommands.WristActuateOpenLoop;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WristFunctionality.IntakeWrist;
import frc.robot.subsystems.WristFunctionality.ShooterWrist;

/** Add your docs here. */
public class ConfigureButtonBindings {
    public ConfigureButtonBindings(
        XboxController m_driverController, XboxController m_operatorController, 
        DriveSubsystem m_robotDrive, Intake m_Intake, Shooter m_Shooter, 
        IntakeWrist m_IntakeWrist, ShooterWrist m_ShooterWrist, Indexer m_Indexer, 
        Climb m_Climb) {
        
         /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its`
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  
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
         new IntakeDump(m_Intake));

    new Trigger(()-> m_operatorController.getLeftY() > 0.15 || m_operatorController.getLeftY() < -0.15).whileTrue(
        new WristActuateOpenLoop(m_IntakeWrist, () -> {
            System.out.println(-MathUtil.applyDeadband(m_operatorController.getLeftY(), OIConstants.kDriveDeadband));
            return -MathUtil.applyDeadband(m_operatorController.getLeftY(), OIConstants.kDriveDeadband);
        }));

    new Trigger(()-> m_operatorController.getRightBumper()).whileTrue(
         new ShooterRecieve(m_Shooter).alongWith(new IndexIntakeToShooter(m_Indexer)));  
    
    new Trigger(()-> m_operatorController.getRightTriggerAxis() > 0).whileTrue(
        new ShooterFire(m_Shooter, m_operatorController::getRightTriggerAxis)
        .alongWith(new IndexIntakeToShooter(m_Indexer)));


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
}
