package frc.robot.subsystems.WristFunctionality;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.LimitSwitchConstants;
import frc.robot.Constants.limitDirection;

public class IntakeWrist extends Wrist{

  private final DigitalInput m_BackwardLimitSwitch;
  public IntakeWrist(double kP, double kI, double kD, double holdConstant, int motorCANID){
    super(kP, kI, kD, holdConstant, motorCANID);
    this.m_BackwardLimitSwitch = new DigitalInput(LimitSwitchConstants.kIntakeWristBack);
  }

  public boolean isTouchingLimitSwitch(){
    return !m_BackwardLimitSwitch.get();
  }

  public limitDirection getLimitSwitchDirection(){
    return limitDirection.BACKWARD;
  }

  public boolean getIsPidInverted(){
    return true;
  }
  //going from upright to the left is positive input

    
}
