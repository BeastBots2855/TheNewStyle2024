package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.LimitSwitchConstants;

public class ShooterWrist extends Wrist{

  private final DigitalInput m_ForwardLimitSwitch;
  private final DigitalInput m_BackwardLimitSwitch;
  public ShooterWrist(double kP, double kI, double kD, double holdConstant, int motorCANID){
    super(kP, kI, kD, holdConstant, motorCANID);
    m_ForwardLimitSwitch = new DigitalInput(LimitSwitchConstants.kIntakeLimit);
    m_BackwardLimitSwitch = new DigitalInput(LimitSwitchConstants.kIntakeWristBack);
  }

  public boolean isTouchingLimitSwitch(){
    return m_ForwardLimitSwitch.get();
  }
    
}
