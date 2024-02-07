package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.LimitSwitchConstants;

public class IntakeWrist extends Wrist{

  private final DigitalInput m_BackwardLimitSwitch;
  public IntakeWrist(double kP, double kI, double kD, double holdConstant, int motorCANID){
    super(kP, kI, kD, holdConstant, motorCANID);
    this.m_BackwardLimitSwitch = new DigitalInput(LimitSwitchConstants.kIntakeWristBack);
  }

  public boolean isTouchingLimitSwitch(){
    return !m_BackwardLimitSwitch.get();
  }
    
}
