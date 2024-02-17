// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private TalonSRX m_LeftClimbMotorTop;
  private TalonSRX m_LeftClimbMotorBottom;
  private TalonSRX m_RightClimbMotorTop;
  private TalonSRX m_RightClimbMotorBottom;
  //private final  m_climbLeftBottom;
  public Climb() {
  m_LeftClimbMotorTop = new TalonSRX(ClimbConstants.kLeftTopCanID);
  m_LeftClimbMotorBottom = new TalonSRX(ClimbConstants.kLeftBottomCanID);
  m_RightClimbMotorTop = new TalonSRX(ClimbConstants.kRightTopCanID);
  m_RightClimbMotorBottom = new TalonSRX(ClimbConstants.kRightBottomCanID);
  
  }

  public void setMotorOutput(double leftOutput, double rightOutput){
    m_LeftClimbMotorTop.set(TalonSRXControlMode.PercentOutput, -leftOutput);
    m_LeftClimbMotorBottom.set(TalonSRXControlMode.PercentOutput, -leftOutput);
    m_RightClimbMotorTop.set(TalonSRXControlMode.PercentOutput, rightOutput);
    m_RightClimbMotorBottom.set(TalonSRXControlMode.PercentOutput, rightOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
