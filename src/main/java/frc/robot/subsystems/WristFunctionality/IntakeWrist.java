// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.WristFunctionality;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeWristConstants;
import frc.robot.Constants.LimitSwitchConstants;

public class IntakeWrist extends SubsystemBase implements Wrist {
  /** Creates a new Wrist. */
  private double m_holdConstant;
  private final CANSparkMax m_wristMotor;
  private final AbsoluteEncoder m_absoluteEncoder;
  private final PIDController m_PidController;
  private boolean m_isPidEnabled = false;
  private final double m_PidTolerance;
  private final DigitalInput m_BackwardLimitSwitch;
  


  public IntakeWrist() {
    m_PidController = new PIDController(0.01, 0, 0.0);
    m_PidTolerance = 2;
    // m_PidController.enableContinuousInput(0, 360);
    m_holdConstant = 0.03;
    m_wristMotor = new CANSparkMax(IntakeWristConstants.IntakeWristCANID, MotorType.kBrushless);
    m_absoluteEncoder = m_wristMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_absoluteEncoder.setPositionConversionFactor(360);
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotor.burnFlash();
    this.m_BackwardLimitSwitch = new DigitalInput(LimitSwitchConstants.kIntakeWristBack);
    
  }

  public void setMotorOutput(double output) {
    if (!isTouchingLimitSwitch()){
      m_wristMotor.set(output);
    } else if (isTouchingLimitSwitch() && output > 0) { 
      m_wristMotor.set(output);
    }else {
      m_wristMotor.set(0);
    }
  }

  public void setSetpoint(double setPoint){
    m_PidController.setSetpoint(setPoint);
  }

  public void enablePid(){
    m_isPidEnabled = true;
  }

  public void disblePid(){
    m_isPidEnabled = false;
    m_wristMotor.set(0);
  }

  public boolean isPidEnabled(){
    return m_isPidEnabled;
  }

  public void runPid(){
  
      double angle = m_absoluteEncoder.getPosition();
      if (angle > 270)
        angle = angle - 360;
      double output = m_PidController.calculate(angle);
      if (!isTouchingLimitSwitch()){
        output += m_holdConstant * Math.cos(m_absoluteEncoder.getPosition());
        // output = m_PidController.getPositionError() > 180 && m_absoluteEncoder.getPosition() > 270 ? -output : output;
        setMotorOutput(output);
      } else {
        setMotorOutput(0);
      }


      // System.out.println("Position: " + m_absoluteEncoder.getPosition());
      // System.out.println("Target: " + m_PidController.getSetpoint());
      // System.out.println("Error: "  );
      // System.out.println("Output: " + output);
  }

  public boolean isWithinPidTolerance(){
    return Math.abs(m_PidController.getSetpoint() - m_absoluteEncoder.getPosition()) < m_PidTolerance;
  }


  public double getAbsoluteEncoderValue(){
    return m_absoluteEncoder.getPosition();
  }
  
  public boolean isTouchingLimitSwitch(){
    return false;
  }


  @Override
   public void periodic() {
      if(m_isPidEnabled) {
          runPid();
      }
      
   }


   

}
