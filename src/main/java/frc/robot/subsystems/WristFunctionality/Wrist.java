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
import frc.robot.Constants.WristLocation;
import frc.robot.Constants.limitDirection;

public interface Wrist{
  /** Creates a new Wrist. */
  public void setMotorOutput(double output);

  public void setSetpoint(double setPoint);

  public void enablePid();

  public void disblePid();

  public void runPid();

  public double getAbsoluteEncoderValue();

  abstract boolean isTouchingLimitSwitch();
  



   

}
