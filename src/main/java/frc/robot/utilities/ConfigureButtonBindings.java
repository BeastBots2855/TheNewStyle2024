// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ClimberClimb;
import frc.robot.Commands.SetIntakeToShooterPosition;
import frc.robot.Commands.CommandGroups.SetIntakeInShooterAmp;
import frc.robot.Commands.IndexCommands.IndexIntakeToShooter;
import frc.robot.Commands.IndexCommands.IndexShooterToIntake;
import frc.robot.Commands.IntakeCommands.IntakeConsume;
import frc.robot.Commands.IntakeCommands.IntakeDump;
import frc.robot.Commands.ShooterCommands.ShooterFire;
import frc.robot.Commands.ShooterCommands.ShooterRescind;
import frc.robot.Commands.WristCommands.IntakeWristClosedLoop;
import frc.robot.Commands.WristCommands.IntakeWristOpenLoop;
import frc.robot.Commands.WristCommands.ShooterWristClosedLoop;
import frc.robot.Commands.WristCommands.ShooterWristOpenLoop;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.WristFunctionality.IntakeWrist;
import frc.robot.Subsystems.WristFunctionality.ShooterWrist;

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
  
    // new JoystickButton(m_driverController, XboxController.Button.kA.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .whileTrue(new InstantCommand(
          m_robotDrive::zeroHeading, 
          m_robotDrive));

    new Trigger(()-> m_operatorController.getLeftTriggerAxis() > 0).whileTrue(
        new IntakeConsume(m_Intake, m_operatorController::getLeftTriggerAxis));

    new Trigger(()-> m_operatorController.getLeftBumper()).whileTrue(
         new IntakeDump(m_Intake));

    new Trigger(()-> m_operatorController.getLeftY() > 0.15 || m_operatorController.getLeftY() < -0.15).whileTrue(
        new IntakeWristOpenLoop(m_IntakeWrist, () -> {
            System.out.println(-MathUtil.applyDeadband(m_operatorController.getLeftY(), OIConstants.kDriveDeadband));
            return MathUtil.applyDeadband(m_operatorController.getLeftY(), OIConstants.kDriveDeadband);
        }));

  
    
    new Trigger(()-> m_operatorController.getRightTriggerAxis() > 0).whileTrue(
        new ShooterFire(m_Shooter, m_operatorController::getRightTriggerAxis));


    new Trigger(()-> m_operatorController.getRightY() > 0.15 || m_operatorController.getRightY() < -0.15).whileTrue(
        new ShooterWristOpenLoop(
                m_ShooterWrist, 
                () ->  MathUtil.applyDeadband(m_operatorController.getRightY(), OIConstants.kDriveDeadband)));

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

        


    // new Trigger(()-> m_operatorController.getAButton()).whileTrue(
    //      new ShooterWristClosedLoop(m_ShooterWrist, 45.0).alongWith(new PrintCommand("PIDEnabled"))); 
    // new Trigger(()-> m_operatorController.getBButton()).whileTrue(
    //      new ShooterWristClosedLoop(m_ShooterWrist, 140).alongWith(new PrintCommand("PIDEnabled"))); 
    // new Trigger(()-> m_operatorController.getPOV() == 180).whileTrue(
    //     new IntakeWristClosedLoop(m_IntakeWrist, 3).alongWith(new PrintCommand("PIDEnabled"))); 
    //   new Trigger(()-> m_operatorController.getPOV() == 0).whileTrue(
    //     new IntakeWristClosedLoop(m_IntakeWrist, 180).alongWith(new PrintCommand("PIDEnabled")));

    new Trigger(()-> m_operatorController.getXButton()).whileTrue(new SetIntakeInShooterAmp(m_ShooterWrist, m_IntakeWrist));

    new Trigger(()-> m_operatorController.getYButton()).whileTrue(new SetIntakeToShooterPosition(m_ShooterWrist, m_IntakeWrist));

    new Trigger(()-> m_operatorController.getBButton()).whileTrue(
         new ShooterWristClosedLoop(m_ShooterWrist, 45).alongWith(new PrintCommand("PIDEnabled"))
         .alongWith(new IntakeWristClosedLoop(m_IntakeWrist, 180))); 
    new Trigger(()-> m_operatorController.getAButton()).whileTrue(
         new ShooterWristClosedLoop(m_ShooterWrist, 131).alongWith(new PrintCommand("PIDEnabled"))
         .alongWith(new IntakeWristClosedLoop(m_IntakeWrist, 180))); 
    new Trigger(()-> m_operatorController.getStartButton()).whileTrue(
        new ShooterWristClosedLoop(m_ShooterWrist, 90).alongWith(new PrintCommand("PIDEnabled"))
        .alongWith(new IntakeWristClosedLoop(m_IntakeWrist, 3))); 
        
    new Trigger(()-> m_operatorController.getPOV(0) == 0)
        .whileTrue(new RunCommand(()->m_Indexer.setMotorOutput(-1), m_Indexer))
        .whileTrue(new RunCommand(()->m_Shooter.setMotorOutput(-1), m_Shooter))
        .whileFalse(new RunCommand(()->m_Indexer.setMotorOutput(0), m_Indexer));

    new Trigger(()-> m_operatorController.getPOV(0) == 180)
        .whileTrue(new RunCommand(()->m_Indexer.setMotorOutput(1), m_Indexer).alongWith(
            new ShooterRescind(m_Shooter)))
        .whileFalse(new RunCommand(()->m_Indexer.setMotorOutput(0), m_Indexer));

    new Trigger(()-> m_operatorController.getRightBumper())
        .whileTrue(new IndexIntakeToShooter(m_Indexer));








    //new Trigger(()-> m_Intake.isTouchingLimitSwitch()).onTrue(new RunCommand(()->m_Indexer.setPrimedForNote()));
    // new Trigger(()-> m_Intake.isTouchingLimitSwitch())
    //     .onTrue(new SequentialCommandGroup(
    //         new RunCommand(()-> {
    //             m_operatorController.setRumble(RumbleType.kBothRumble, 1);
    //             m_driverController.setRumble(RumbleType.kBothRumble, 1);}), 
    //         new WaitCommand(2),
    //         new RunCommand(()-> {
    //             m_operatorController.setRumble(RumbleType.kBothRumble, 0);
    //             m_driverController.setRumble(RumbleType.kBothRumble, 0);})));

    new Trigger(()-> m_Intake.isTouchingLimitSwitch()).onTrue(
        new ParallelCommandGroup(
            new IntakeWristClosedLoop(m_IntakeWrist, 180),
            new ShooterWristClosedLoop(m_ShooterWrist, 140))
        .until(()-> m_IntakeWrist.isWithinPidTolerance() && m_ShooterWrist.isWithinPidTolerance())
        .andThen(
            new ParallelCommandGroup(
                new InstantCommand(()-> m_IntakeWrist.setMotorOutput(0), m_IntakeWrist),
                new InstantCommand(()-> m_ShooterWrist.setMotorOutput(0), m_ShooterWrist))
            ).andThen(
            new ParallelDeadlineGroup(
                new WaitCommand(0.38),
                new IntakeDump(m_Intake),
                new IndexIntakeToShooter(m_Indexer))
            ));
    
//131.60

    
    new Trigger(()-> m_driverController.getRightTriggerAxis() != 0).whileTrue(new ClimberClimb(m_Climb, ()-> m_driverController.getRightTriggerAxis(), m_robotDrive::getPitch));
    new Trigger(()-> m_driverController.getLeftTriggerAxis() != 0).whileTrue(new ClimberClimb(m_Climb, ()-> -m_driverController.getLeftTriggerAxis(), m_robotDrive::getPitch));

    
        

        NamedCommands.registerCommand("ShooterFire", new ShooterFire(m_Shooter, ()-> 0.5));
        NamedCommands.registerCommand("ShooterRescind", new ShooterRescind(m_Shooter));
        NamedCommands.registerCommand("IndexIntakeToShooter", new IndexIntakeToShooter(m_Indexer));
        NamedCommands.registerCommand("IndexShooterToIntake", new IndexShooterToIntake(m_Indexer));
        NamedCommands.registerCommand("IntakeConsume", new IntakeConsume(m_Intake, ()-> 0.5));
        NamedCommands.registerCommand("IntakeDump", new IntakeDump(m_Intake));
        NamedCommands.registerCommand("ShooterRecieve", new ShooterWristClosedLoop(m_ShooterWrist, 131.6));
        NamedCommands.registerCommand("ShooterToSpeaker", new ShooterWristClosedLoop(m_ShooterWrist, 131.6));
        NamedCommands.registerCommand("ShooterToAmp", new ShooterWristClosedLoop(m_ShooterWrist, 45));
        NamedCommands.registerCommand("IntakeToGround", new IntakeWristClosedLoop(m_IntakeWrist, 3));
        NamedCommands.registerCommand("IntakeToShooter", new IntakeWristClosedLoop(m_IntakeWrist, 180));

    }

}
