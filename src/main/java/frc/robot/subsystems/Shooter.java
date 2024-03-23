// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final CANSparkMax m_ShooterMotor1;
  private final CANSparkMax m_ShooterMotor2;
  private final RelativeEncoder mEncoder;
  public Shooter() {
    m_ShooterMotor1 = new CANSparkMax(ShooterConstants.ShooterMotorCANID, MotorType.kBrushless);
    m_ShooterMotor2 = new CANSparkMax(ShooterConstants.ShooterMotor2CANID, MotorType.kBrushless); 
    mEncoder = m_ShooterMotor1.getEncoder();
  }

  public void setMotorOutput(double output){
    m_ShooterMotor1.set(output);
    m_ShooterMotor2.set(output);
  }

  public double getVelocity(){
    return mEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
