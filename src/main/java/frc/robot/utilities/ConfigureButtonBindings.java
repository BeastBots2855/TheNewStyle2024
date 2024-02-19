// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimberClimb;
import frc.robot.commands.IndexCommands.IndexIntakeToShooter;
import frc.robot.commands.IndexCommands.IndexShooterToIntake;
import frc.robot.commands.IntakeCommands.IntakeConsume;
import frc.robot.commands.IntakeCommands.IntakeDump;
import frc.robot.commands.MechanismSequences.GroundNoteToIndexer;
import frc.robot.commands.MechanismSequences.SetClimbPosition;
import frc.robot.commands.MechanismSequences.SetIntakeGroundShooterIn;
import frc.robot.commands.MechanismSequences.SetIntakeInShooterAmp;
import frc.robot.commands.MechanismSequences.SetIntakeInShooterIn;
import frc.robot.commands.MechanismSequences.SetIntakeInShooterSpeaker;
import frc.robot.commands.ShooterCommands.ShooterFire;
import frc.robot.commands.ShooterCommands.ShooterRescind;
import frc.robot.commands.WristCommands.IntakeWristClosedLoop;
import frc.robot.commands.WristCommands.IntakeWristOpenLoop;
import frc.robot.commands.WristCommands.ShooterWristClosedLoop;
import frc.robot.commands.WristCommands.ShooterWristOpenLoop;
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
        
    //Driver Bindings
        //Driving Bindings
            //Movement
            m_robotDrive.setDefaultCommand(
                new RunCommand(
                    () -> m_robotDrive.drive(
                        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                        true, true),
                    m_robotDrive));
            //Set X
            new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                .whileTrue(new RunCommand(
                    () -> m_robotDrive.setX(),
                    m_robotDrive));
            //Reset Heading
            new JoystickButton(m_driverController, XboxController.Button.kStart.value)
                .whileTrue(new InstantCommand(
                m_robotDrive::zeroHeading, 
                m_robotDrive));

        
        //Climb Bindings
            //Activate Climb
            new Trigger(()-> m_driverController.getRightTriggerAxis() != 0).whileTrue(new ClimberClimb(m_Climb, ()-> m_driverController.getRightTriggerAxis(), m_robotDrive::getPitch));
            //Reverse Climb
            new Trigger(()-> m_driverController.getLeftTriggerAxis() != 0).whileTrue(new ClimberClimb(m_Climb, ()-> -m_driverController.getLeftTriggerAxis(), m_robotDrive::getPitch));



    //OperatorJoysticks

        //Open Loop Commands
            //Intake
               //Intake Consume
                new Trigger(()-> m_operatorController.getLeftTriggerAxis() > 0).whileTrue(
                    new IntakeConsume(m_Intake, m_operatorController::getLeftTriggerAxis));
                //IntakeRelease
                new Trigger(()-> m_operatorController.getLeftBumper()).whileTrue(
                    new IntakeDump(m_Intake));
            //Wrists
                //IntakeWrist
                new Trigger(()-> m_operatorController.getLeftY() > 0.15 || m_operatorController.getLeftY() < -0.15).whileTrue(
                    new IntakeWristOpenLoop(m_IntakeWrist, () -> MathUtil.applyDeadband(m_operatorController.getLeftY(), OIConstants.kDriveDeadband)));
                //ShooterWrist
                new Trigger(()-> m_operatorController.getRightY() > 0.15 || m_operatorController.getRightY() < -0.15).whileTrue(
                    new ShooterWristOpenLoop(m_ShooterWrist, () ->  MathUtil.applyDeadband(m_operatorController.getRightY(), OIConstants.kDriveDeadband)));
            //Shooters
                //ShooterFire
                new Trigger(()-> m_operatorController.getRightTriggerAxis() > 0).whileTrue(
                    new ShooterFire(m_Shooter, m_operatorController::getRightTriggerAxis));
                //Run Indexer and Shooter Backward
                    new Trigger(()-> m_operatorController.getPOV(0) == 0)
                        .whileTrue(new RunCommand(()->m_Indexer.setMotorOutput(-1), m_Indexer))
                        .whileTrue(new RunCommand(()->m_Shooter.setMotorOutput(-1), m_Shooter))
                        .whileFalse(new RunCommand(()->m_Indexer.setMotorOutput(0), m_Indexer));
                //Run Indexer and Shooter Forward
                    new Trigger(()-> m_operatorController.getPOV(0) == 180)
                        .whileTrue(new RunCommand(()->m_Indexer.setMotorOutput(1), m_Indexer).alongWith(
                            new ShooterRescind(m_Shooter)))
                        .whileFalse(new RunCommand(()->m_Indexer.setMotorOutput(0), m_Indexer));
                //Run Indexer Forward
                    new Trigger(()-> m_operatorController.getRightBumper())
                        .whileTrue(new IndexIntakeToShooter(m_Indexer));
        //Closed Loop Commands
            //SetIntakeGroundShooterIn
            new Trigger(()-> m_operatorController.getXButton()).onTrue(new SetIntakeGroundShooterIn(m_ShooterWrist, m_IntakeWrist));
            //SetIntakeInShooterIn
            new Trigger(()-> m_operatorController.getYButton()).onTrue(new SetIntakeInShooterIn(m_ShooterWrist, m_IntakeWrist));
            //SetIntakeInShooterAmp
            new Trigger(()-> m_operatorController.getBButton()).onTrue(new SetIntakeInShooterAmp(m_ShooterWrist, m_IntakeWrist));
            //SetIntakeInShooterSpeaker
            new Trigger(()-> m_operatorController.getAButton()).whileTrue(new SetIntakeInShooterSpeaker(m_ShooterWrist, m_IntakeWrist));
            //SetClimbPosition
            new Trigger(()-> m_operatorController.getStartButton()).whileTrue(new SetClimbPosition(m_ShooterWrist, m_IntakeWrist));


    //Non-Driver Controlled Actions
        new Trigger(()-> m_Intake.isTouchingLimitSwitch()).onTrue(new GroundNoteToIndexer(m_IntakeWrist, m_ShooterWrist, m_Intake, m_Indexer));
        

    //The Forsaken One   
    // new Trigger(()-> true).whileTrue(
    //     new RunCommand(()-> {
    //         if (m_Intake.shouldRumble()){
    //             m_operatorController.setRumble(RumbleType.kBothRumble, 1);
    //             m_driverController.setRumble(RumbleType.kBothRumble, 1);
    //         } else {
    //             m_operatorController.setRumble(RumbleType.kBothRumble, 0);
    //             m_driverController.setRumble(RumbleType.kBothRumble, 0);
    //         }
    //     }));
    

    
    
        

        NamedCommands.registerCommand("ShooterFire", new ShooterFire(m_Shooter, ()-> 0.5));
        NamedCommands.registerCommand("ShooterRescind", new ShooterRescind(m_Shooter));
        NamedCommands.registerCommand("IndexIntakeToShooter", new IndexIntakeToShooter(m_Indexer));
        NamedCommands.registerCommand("IndexShooterToIntake", new IndexShooterToIntake(m_Indexer));
        NamedCommands.registerCommand("IntakeConsume", new IntakeConsume(m_Intake, ()-> 0.5));
        NamedCommands.registerCommand("IntakeDump", new IntakeDump(m_Intake));
        NamedCommands.registerCommand("ShooterRecieve", new ShooterWristClosedLoop(m_ShooterWrist, 140));
        NamedCommands.registerCommand("ShooterToSpeaker", new ShooterWristClosedLoop(m_ShooterWrist, 131.6));
        NamedCommands.registerCommand("ShooterToAmp", new ShooterWristClosedLoop(m_ShooterWrist, 45));
        NamedCommands.registerCommand("IntakeToGround", new IntakeWristClosedLoop(m_IntakeWrist, 3));
        NamedCommands.registerCommand("IntakeToShooter", new IntakeWristClosedLoop(m_IntakeWrist, 180));

    }

}
