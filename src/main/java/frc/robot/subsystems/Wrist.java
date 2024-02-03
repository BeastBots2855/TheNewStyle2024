// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Wrist extends PIDSubsystem {
  /** Creates a new Wrist. */
  private double m_holdConstant;
  private final CANSparkMax m_wristMotor;
  private final AbsoluteEncoder m_absoluteEncoder;



  public Wrist(double kP, double kI, double kD, double holdConstant, int motorCANID) {

    super(new PIDController(kP, kI, kD));
    m_holdConstant = holdConstant;
    m_wristMotor = new CANSparkMax(motorCANID, MotorType.kBrushless);
    m_absoluteEncoder = m_wristMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_absoluteEncoder.setPositionConversionFactor(360);
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotor.burnFlash();
    
  }



  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    //add feed forward to the output
    output += m_holdConstant * Math.cos(m_absoluteEncoder.getPosition());
    m_wristMotor.set(output);
  }



  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_absoluteEncoder.getPosition();
  }


  public void setMotorOutput(double output) {
    m_wristMotor.set(output);
  }



}
