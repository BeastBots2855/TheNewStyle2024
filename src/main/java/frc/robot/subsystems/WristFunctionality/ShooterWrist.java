package frc.robot.subsystems.WristFunctionality;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.LimitSwitchConstants;
import frc.robot.Constants.limitDirection;

public class ShooterWrist extends Wrist{

  private final DigitalInput m_ForwardLimitSwitch;
  public ShooterWrist(double kP, double kI, double kD, double holdConstant, int motorCANID){
    super(kP, kI, kD, holdConstant, motorCANID);
    m_ForwardLimitSwitch = new DigitalInput(LimitSwitchConstants.kShooterWristBack);
  }

  public boolean isTouchingLimitSwitch(){
    return !m_ForwardLimitSwitch.get();
  }

  public limitDirection getLimitSwitchDirection(){
    return limitDirection.FORWARD;
  }

  public boolean getIsPidInverted(){
    return true;
  }


  
    
}
