// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

/** Add your docs here. */
public class Indexer extends SubsystemBase{
  /** Creates a new Shooter. */
  private final CANSparkMax m_IndexerMotor;
  public Indexer() {
    m_IndexerMotor = new CANSparkMax(IndexerConstants.IndexerMotorCANID, MotorType.kBrushless);
  }

  public void setMotorOutput(double output){
    m_IndexerMotor.set(output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
