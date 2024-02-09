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
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.limitDirection;

public abstract class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private double m_holdConstant;
  private final CANSparkMax m_wristMotor;
  private final AbsoluteEncoder m_absoluteEncoder;
  private final PIDController m_PidController;
  private boolean m_isPidEnabled = false;



  public Wrist(double kP, double kI, double kD, double holdConstant, int motorCANID) {

    m_PidController = new PIDController(kP, kI, kD);
    m_PidController.enableContinuousInput(0, 360);
    m_holdConstant = holdConstant;
    m_wristMotor = new CANSparkMax(motorCANID, MotorType.kBrushless);
    m_absoluteEncoder = m_wristMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_absoluteEncoder.setPositionConversionFactor(360);
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotor.burnFlash();
  }

  public void setMotorOutput(double output) {
    if (!isTouchingLimitSwitch()){
      m_wristMotor.set(output);
    } else if (isTouchingLimitSwitch() && getLimitSwitchDirection() == limitDirection.FORWARD && output < 0) { 
      m_wristMotor.set(output);
    } else if (isTouchingLimitSwitch() && getLimitSwitchDirection() == limitDirection.BACKWARD && output > 0) {
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
      output = getIsPidInverted() ? -output : output;
      output = Math.abs(m_PidController.getSetpoint()) - Math.abs(m_absoluteEncoder.getPosition() - 15) > 180 ? -output : output;
      if (!isTouchingLimitSwitch()){
        output += m_holdConstant * Math.cos(m_absoluteEncoder.getPosition());
        setMotorOutput(output);
      } else {
        setMotorOutput(0);
      }
      System.out.println("Position: " + m_absoluteEncoder.getPosition());
      System.out.println("Output: " + output);

  }

  @Override
   public void periodic() {
    if(m_isPidEnabled) {
      runPid();
    }
   }

  abstract boolean isTouchingLimitSwitch();
  abstract limitDirection getLimitSwitchDirection();
  abstract boolean getIsPidInverted();
  



   

}
