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

        m_topSparkMax = new CANSparkMax(31, MotorType.kBrushless);
        m_bottomSparkMax = new CANSparkMax(33, MotorType.kBrushless);

        m_topMotorFeedforward = new SimpleMotorFeedforward(0.1799, 0.00033);
        m_bottomMotorFeedforward = new SimpleMotorFeedforward(0.2188, 0.00036);

        m_topSparkMax.restoreFactoryDefaults();
        m_bottomSparkMax.restoreFactoryDefaults();

        m_topMotorEncoder = m_topSparkMax.getEncoder();
        m_bottomMotorEncoder = m_bottomSparkMax.getEncoder();

        m_topPidController = m_topSparkMax.getPIDController();
        m_bottomPidController = m_bottomSparkMax.getPIDController();

        // m_bottomSparkMax.setInverted(true);


        m_topPidController.setP(0.00000681);
        m_topPidController.setI(0.0000000015);
        m_topPidController.setD(0);
        m_topPidController.setFF(0.00016);
        m_topPidController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

        m_bottomPidController.setP(0.00000681);
        m_bottomPidController.setI(0.0000000015);
        m_bottomPidController.setD(0);
        m_bottomPidController.setFF(0.00016);
        m_bottomPidController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);


        m_bottomPidController.setFeedbackDevice(m_bottomMotorEncoder);
        m_topPidController.setFeedbackDevice(m_topMotorEncoder);

        m_topSparkMax.burnFlash();
        m_bottomSparkMax.burnFlash();
      }

      public void setMotorVelocities(double targetSpeed){
        m_topPidController.setReference(5000, CANSparkMax.ControlType.kVelocity, 0, m_topMotorFeedforward.calculate(targetSpeed));
        m_bottomPidController.setReference(5000, CANSparkMax.ControlType.kVelocity, 0, m_bottomMotorFeedforward.calculate(targetSpeed));
      }

      @Override
      public void periodic() {
        // This method will be called once per scheduler run
        System.out.println(m_topMotorEncoder.getVelocity());
        
      }
}
