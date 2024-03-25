// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimberClimb;
import frc.robot.commands.IndexCommands.IndexIntakeToShooter;
import frc.robot.commands.IndexCommands.IndexShooterToIntake;
import frc.robot.commands.IntakeCommands.IntakeConsume;
import frc.robot.commands.IntakeCommands.IntakeDump;
import frc.robot.commands.ShooterCommands.NewShooterFire;
import frc.robot.commands.ShooterCommands.NewShooterFire;
import frc.robot.commands.ShooterCommands.NewShooterRescind;
import frc.robot.commands.Vision.FeederLockOn;
import frc.robot.commands.Vision.NoteLockOn;
import frc.robot.commands.Vision.SpeakerLockOn;
import frc.robot.commands.WristCommands.IntakeWristClosedLoop;
import frc.robot.commands.WristCommands.IntakeWristOpenLoop;
import frc.robot.commands.WristCommands.ShooterWristClosedLoop;
import frc.robot.commands.WristCommands.ShooterWristOpenLoop;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OldShooter;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.subsystems.WristFunctionality.IntakeWrist;
import frc.robot.subsystems.WristFunctionality.ShooterWrist;
import frc.robot.commands.AutoCommands.AutoAmpScore;
import frc.robot.commands.AutoCommands.AutoIntake;
import frc.robot.commands.AutoCommands.InitialShot;
import frc.robot.commands.AutoCommands.SetIntakeGround;
import frc.robot.commands.Automatted.AutoAimWithShooterAngle;
import frc.robot.commands.MechanismSequences.GroundNoteToIndexer;
import frc.robot.commands.MechanismSequences.SetClimbPosition;
import frc.robot.commands.MechanismSequences.SetIntakeGroundShooterIn;
import frc.robot.commands.MechanismSequences.SetIntakeInShooterAmp;
import frc.robot.commands.MechanismSequences.SetIntakeInShooterIn;
import frc.robot.commands.MechanismSequences.SetIntakeInShooterSpeaker;
import frc.robot.commands.LedCommands.RAINBOWS;
import frc.robot.commands.LedCommands.SetLights;
import frc.robot.Constants.Colors;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.NewShooter;

/** Add your docs here. */
public class ConfigureButtonBindings {
    public ConfigureButtonBindings(
        XboxController m_driverController, XboxController m_operatorController, 
        DriveSubsystem m_robotDrive, Intake m_Intake, 
        IntakeWrist m_IntakeWrist, ShooterWrist m_ShooterWrist, Indexer m_Indexer, 
        Climb m_Climb,LED m_Led, Autos m_Autos, NewShooter m_Shooter) {
        
    /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its`
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .whileTrue(new InstantCommand(
          m_robotDrive::zeroHeading, 
          m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .whileTrue(new InstantCommand(
          m_robotDrive::zeroHeading, 
          m_robotDrive));

    // new JoystickButton(m_driverController, XboxController.Button.kStart.value)
    //     .onTrue(null)(new InstantCommand(
    //     ()-> m_robotDrive.zeroHeading()), 
    //     m_robotDrive);

        //LEDs
            // new Trigger(()->m_driverController.getAButton()).onTrue(new RAINBOWS(m_Led));
            // new Trigger(()->m_driverController.getBButton()).onTrue(new SetLights(m_Led, Colors.green));
            // new Trigger(()->m_driverController.getYButton()).onTrue(new SetLights(m_Led, Colors.yellow));
            // new Trigger(()->m_driverController.getXButton()).onTrue(new SetLights(m_Led, Colors.red));
        

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
                    new NewShooterFire(m_Shooter, m_operatorController::getRightTriggerAxis));
                //Run Indexer and Shooter Backward
                    new Trigger(()-> m_operatorController.getPOV(0) == 0)
                        .whileTrue(new RunCommand(()->m_Indexer.setMotorOutput(-1), m_Indexer))
                        .whileTrue(new RunCommand(()->m_Shooter.setRPMForAmp(), m_Shooter))
                        .whileFalse(new RunCommand(()->m_Indexer.setMotorOutput(0), m_Indexer));

                //New shooter velocity
                new Trigger(()-> m_operatorController.getPOV(0) == 90)
                        .whileTrue(new NewShooterFire(m_Shooter, ()->1.0));

                //Run Indexer and Shooter Forward
                    new Trigger(()-> m_operatorController.getPOV(0) == 180)
                        .whileTrue(new RunCommand(()->m_Indexer.setMotorOutput(1), m_Indexer).alongWith(
                            new NewShooterRescind(m_Shooter, ()-> 0.2)))
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
        new Trigger(()-> m_Intake.isTouchingLimitSwitch() && !DriverStation.isAutonomousEnabled()).onTrue(new GroundNoteToIndexer(m_IntakeWrist, m_ShooterWrist, m_Intake, m_Indexer));
        
        //Set lights to yellow if trying to track a note and can see a note
        new Trigger(()-> PhotonVision.canTrustNoteData() && m_driverController.getLeftBumper()).whileTrue(new SetLights(m_Led, Colors.yellow)); 
        //Set lights to red if trying to track a note and can't see a note
        new Trigger(()->!PhotonVision.canTrustNoteData() && m_driverController.getLeftBumper()).whileTrue(new SetLights(m_Led, Colors.red));
        
        //Set lights to green when contacting note
        new Trigger(()-> m_Intake.isTouchingLimitSwitch()).whileTrue(new SetLights(m_Led, Colors.green)).whileFalse(new RAINBOWS(m_Led));
        //The Forsaken One   
        new Trigger(()-> m_Intake.isTouchingLimitSwitch()).onTrue(
            new RunCommand(()-> {
                    m_operatorController.setRumble(RumbleType.kBothRumble, 1);
                    m_driverController.setRumble(RumbleType.kBothRumble, 1);}))
            .onFalse(
                new RunCommand(()-> {
                    m_operatorController.setRumble(RumbleType.kBothRumble, 0);
                    m_driverController.setRumble(RumbleType.kBothRumble, 0);})
            );


    






//Experimental Vision
    

    //Note Lock on
     new Trigger(()-> m_driverController.getLeftBumper() && PhotonVision.canTrustNoteData()).whileTrue(
         new NoteLockOn(
             m_robotDrive, 
             ()-> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
             ()-> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)));

            
   // Speaker Lock on 
     new Trigger(()-> m_driverController.getRightBumper()).whileTrue(
         new SpeakerLockOn(
             m_robotDrive, 
             ()-> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
             ()-> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)));
   

     new Trigger(()-> m_driverController.getRightBumper()).whileTrue(
         new AutoAimWithShooterAngle(
             m_robotDrive,
             m_ShooterWrist,
             ()-> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
             ()-> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)));

    //Feeder Lock on 
    new Trigger(()-> m_driverController.getRightBumper()).whileTrue(
        new FeederLockOn(
            m_robotDrive,
            ()-> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            ()-> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)));

    //Amp Auto Score
    // new Trigger(()-> m_driverController.getAButton()).onTrue(
    //     new AutoAmpScore(m_robotDrive, m_ShooterWrist, m_IntakeWrist, m_Shooter));
    //Set lights to yellow if trying to track a note and can see a note
    new Trigger(()-> PhotonVision.canTrustNoteData() && m_driverController.getLeftBumper()).whileTrue(new SetLights(m_Led, Colors.yellow)); 

    //Set lights to red if trying to track a note and can't see a note
    new Trigger(()->!PhotonVision.canTrustNoteData() && m_driverController.getLeftBumper()).whileTrue(new SetLights(m_Led, Colors.red));
          
            

//side shooter set point 
//path planner start of auto Shooter recieve 


        NamedCommands.registerCommand("ShooterFire", new NewShooterFire(m_Shooter, ()-> 0.5));
        NamedCommands.registerCommand("ShooterFireFast", new NewShooterFire(m_Shooter, ()-> 1.0));
        NamedCommands.registerCommand("ShooterRescind", new NewShooterRescind(m_Shooter, ()-> 0.2));
        NamedCommands.registerCommand("IndexIntakeToShooter", new IndexIntakeToShooter(m_Indexer));
        NamedCommands.registerCommand("IndexShooterToIntake", new IndexShooterToIntake(m_Indexer));
        NamedCommands.registerCommand("IntakeConsume", new IntakeConsume(m_Intake, ()-> 0.5));
        NamedCommands.registerCommand("AutoIntake", new AutoIntake(m_IntakeWrist, m_ShooterWrist, m_Intake, m_Indexer));
        NamedCommands.registerCommand("IntakeDump", new IntakeDump(m_Intake));
        NamedCommands.registerCommand("ShooterRecieve", new ShooterWristClosedLoop(m_ShooterWrist, 140));
        NamedCommands.registerCommand("ShooterToSpeaker", new ShooterWristClosedLoop(m_ShooterWrist, 131.6));
        NamedCommands.registerCommand("ShooterToAmp", new ShooterWristClosedLoop(m_ShooterWrist, 45));
        NamedCommands.registerCommand("IntakeToGround", new IntakeWristClosedLoop(m_IntakeWrist, 3));
        NamedCommands.registerCommand("SetIntakeGroundShooterIn", new SetIntakeGroundShooterIn(m_ShooterWrist, m_IntakeWrist));
        NamedCommands.registerCommand("SetIntakeGround", new SetIntakeGround(m_IntakeWrist));
        NamedCommands.registerCommand("SetIntakeInShooterIn", new SetIntakeInShooterIn(m_ShooterWrist, m_IntakeWrist));
        NamedCommands.registerCommand("SetIntakeInShooterAmp", new SetIntakeInShooterAmp(m_ShooterWrist, m_IntakeWrist));
        NamedCommands.registerCommand("SetIntakeInShooterSpeaker", new SetIntakeInShooterSpeaker(m_ShooterWrist, m_IntakeWrist));
        NamedCommands.registerCommand("InitialShot", new InitialShot());        
        m_robotDrive.configureAutoBuilder();
        m_Autos.mapCommands();

        
        
    }

}
