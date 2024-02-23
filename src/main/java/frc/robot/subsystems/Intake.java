// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimitSwitchConstants;
import frc.robot.subsystems.LED;

public class Intake extends SubsystemBase {
  /** Creates a new Shooter. */
  private final CANSparkMax m_IntakeMotor;
  private final DigitalInput m_IntakeButton;
  public Intake() {
    m_IntakeMotor = new CANSparkMax(IntakeConstants.IntakeMotorCANID, MotorType.kBrushless);
    m_IntakeButton = new DigitalInput(LimitSwitchConstants.kIntakeButton);
    
  }

  public void setMotorOutput(double output){
    if (m_IntakeButton.get() && output > 0) {
        m_IntakeMotor.set(output);
    } else if (!m_IntakeButton.get() && output > 0) {
      m_IntakeMotor.set(0);
      
     
    } else {
      m_IntakeMotor.set(output);
    }
      
   
  }

  public void disableMotor(){
    m_IntakeMotor.set(0);
  }

  public boolean isTouchingLimitSwitch(){
    return !m_IntakeButton.get();
  }

  public boolean shouldRumble(){
    return !m_IntakeButton.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
