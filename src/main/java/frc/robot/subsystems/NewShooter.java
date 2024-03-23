// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;

public class NewShooter extends SubsystemBase {
  /** Creates a new NewShooter. */
  private final CANSparkMax m_topSparkMax;
  private final CANSparkMax m_bottomSparkMax;
  private final RelativeEncoder m_topMotorEncoder;
  private final RelativeEncoder m_bottomMotorEncoder;
  private final SimpleMotorFeedforward m_topMotorFeedforward;
  private final SimpleMotorFeedforward m_bottomMotorFeedforward;
  private final SparkPIDController m_topPidController;
  private final SparkPIDController m_bottomPidController;
  public NewShooter() {

        m_topSparkMax = new CANSparkMax(ShooterConstants.ShooterMotorCANID, MotorType.kBrushless);
        m_bottomSparkMax = new CANSparkMax(ShooterConstants.ShooterMotor2CANID, MotorType.kBrushless);

        m_topMotorFeedforward = new SimpleMotorFeedforward(0.1799, 0.00033);
        m_bottomMotorFeedforward = new SimpleMotorFeedforward(0.2188, 0.00036);

        m_topSparkMax.restoreFactoryDefaults();
        m_bottomSparkMax.restoreFactoryDefaults();

        m_topMotorEncoder = m_topSparkMax.getEncoder();
        m_bottomMotorEncoder = m_bottomSparkMax.getEncoder();

        m_topPidController = m_topSparkMax.getPIDController();
        m_bottomPidController = m_bottomSparkMax.getPIDController();

        m_bottomPidController.setFeedbackDevice(m_bottomMotorEncoder);
        m_topPidController.setFeedbackDevice(m_topMotorEncoder);

        

        //PID contants for speaker
        m_topPidController.setP(0.00000681, 0);
        m_topPidController.setI(0.0000000015, 0);
        m_topPidController.setD(0, 0);
        m_topPidController.setFF(0.00016, 0);
        m_topPidController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

        m_bottomPidController.setP(0.00000681, 0);
        m_bottomPidController.setI(0.0000000015, 0);
        m_bottomPidController.setD(0, 0);
        m_bottomPidController.setFF(0.00016, 0);
        m_bottomPidController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);


        //PID contants for amp
        m_topPidController.setP(0.00000681, 1);
        m_topPidController.setI(0.0000000015, 1);
        m_topPidController.setD(0, 1);
        m_topPidController.setFF(0.00016, 1);
        m_topPidController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

        m_bottomPidController.setP(0.00000681, 1);
        m_bottomPidController.setI(0.0000000015, 1);
        m_bottomPidController.setD(0, 1);
        m_bottomPidController.setFF(0.00016, 1);
        m_bottomPidController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);


        //Idle constants if I want them
        m_topPidController.setP(0.00000681, 2);
        m_topPidController.setI(0.0000000015, 2);
        m_topPidController.setD(0, 2);
        m_topPidController.setFF(0.00016, 2);
        m_topPidController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

        m_bottomPidController.setP(0.00000681, 2);
        m_bottomPidController.setI(0.0000000015, 2);
        m_bottomPidController.setD(0, 2);
        m_bottomPidController.setFF(0.00016, 2);
        m_bottomPidController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

      

        m_topSparkMax.burnFlash();
        m_bottomSparkMax.burnFlash();
      }

      public void setMotorRPM(double targetSpeed){
        m_topPidController.setReference(targetSpeed, CANSparkMax.ControlType.kVelocity, 0, m_topMotorFeedforward.calculate(targetSpeed));
        m_bottomPidController.setReference(targetSpeed, CANSparkMax.ControlType.kVelocity, 0, m_bottomMotorFeedforward.calculate(targetSpeed));
      }

      public void setRPMForSpeaker(){
        m_topPidController.setReference(ShooterConstants.kSpeakerRPM, CANSparkMax.ControlType.kVelocity, 0, m_topMotorFeedforward.calculate(ShooterConstants.kSpeakerRPM));
        m_bottomPidController.setReference(ShooterConstants.kSpeakerRPM, CANSparkMax.ControlType.kVelocity, 0, m_bottomMotorFeedforward.calculate(ShooterConstants.kSpeakerRPM));
      }

      public void setRPMForAmp(){
        m_topPidController.setReference(ShooterConstants.kAmpRPM, CANSparkMax.ControlType.kVelocity, 1, m_topMotorFeedforward.calculate(ShooterConstants.kAmpRPM));
        m_bottomPidController.setReference(ShooterConstants.kAmpRPM, CANSparkMax.ControlType.kVelocity, 1, m_bottomMotorFeedforward.calculate(ShooterConstants.kAmpRPM));
      }

      public double getTopMotorRPM() {
        return m_topMotorEncoder.getVelocity();
      }

      public double getBottomMotorRPM() {
        return m_topMotorEncoder.getVelocity();
      }

      public void disableShooter(){
        m_topPidController.setReference(0, CANSparkMax.ControlType.kVelocity, 0, m_topMotorFeedforward.calculate(0));
        m_bottomPidController.setReference(0, CANSparkMax.ControlType.kVelocity, 0, m_bottomMotorFeedforward.calculate(0));
      }

      @Override
      public void periodic() {
        // This method will be called once per scheduler run
        
      }
}
