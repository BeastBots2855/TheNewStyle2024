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
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeWristConstants;
import frc.robot.Constants.LimitSwitchConstants;
import frc.robot.Constants.WristLocation;
import frc.robot.Constants.limitDirection;

public class IntakeWrist extends SubsystemBase implements Wrist {
  /** Creates a new Wrist. */
  private double m_holdConstant;
  private final CANSparkMax m_wristMotor;
  private final AbsoluteEncoder m_absoluteEncoder;
  private final PIDController m_PidController;
  private boolean m_isPidEnabled = false;
  private final DigitalInput m_BackwardLimitSwitch;


  public IntakeWrist() {
    m_PidController = new PIDController(0.01, 0, 0.001);
    m_PidController.enableContinuousInput(0, 360);
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
  }

  public void runPid(){
    
      
      double output = -m_PidController.calculate(m_absoluteEncoder.getPosition());
      if (!isTouchingLimitSwitch()){
        output += m_holdConstant * Math.cos(m_absoluteEncoder.getPosition());
        setMotorOutput(output);
      } else {
        setMotorOutput(0);
      }
      System.out.println("Position: " + m_absoluteEncoder.getPosition());
      System.out.println("Target: " + m_PidController.getSetpoint());
      System.out.println("Error: "  );
      System.out.println("Output: " + output);
  }


  public double getAbsoluteEncoderValue(){
    return m_absoluteEncoder.getPosition();
  }
  
  public boolean isTouchingLimitSwitch(){
    return !m_BackwardLimitSwitch.get();
  }


  @Override
   public void periodic() {
      if(m_isPidEnabled) {
          runPid();
      }
   }


   

}
