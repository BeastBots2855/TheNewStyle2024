// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.LimitSwitchConstants;

/** Add your docs here. */
public class Indexer extends SubsystemBase{
  /** Creates a new Shooter. */
  private final DigitalInput m_IndexerLimitSwitch;
  private final CANSparkMax m_IndexerMotor;
  private boolean m_IsPrimedForNewNote;
  public Indexer() {
    m_IndexerMotor = new CANSparkMax(IndexerConstants.IndexerMotorCANID, MotorType.kBrushless);
    m_IndexerLimitSwitch = new DigitalInput(LimitSwitchConstants.kIndexerSwitch);
  }

  public void setMotorOutput(double output){
    m_IndexerMotor.set(output);
  }

  public boolean isTouchingLimitSwitch(){
    return !m_IndexerLimitSwitch.get();
  }

  public boolean getIsPrimed(){
    return m_IsPrimedForNewNote;
    
  }

  public void setPrimedForNote(){
    m_IsPrimedForNewNote = true;
  }

  public void setNotReadyForNote(){
    m_IsPrimedForNewNote = false;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isTouchingLimitSwitch()){
      m_IsPrimedForNewNote = false;
    }
  }
}
