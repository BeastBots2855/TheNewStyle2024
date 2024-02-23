// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.RGBColor;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  int m_firstLight =1;
  public LED() {
    
    m_led = new AddressableLED(0);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(46);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    for(int i = 0; i < m_ledBuffer.getLength(); i++ ){
      m_ledBuffer.setRGB(i, 255, 100, 0);
    }
    m_led.setData(m_ledBuffer);
    System.out.println("LED's on");
    System.out.println(m_ledBuffer.getLength());
    System.out.println(m_ledBuffer.getLED(0));
  
  
  }
  //getColor

   public void setColor(int red, int green, int blue){
   //m_led.setData(m_ledBuffer);
    // m_led.start();
     for(int i = 0; i< m_ledBuffer.getLength(); i++){
    
        m_ledBuffer.setRGB(i, red, green, blue);
      
     }
     m_led.setData(m_ledBuffer);
   }

  public void setColor(RGBColor m_color){
      setColor(m_color.getRed(), m_color.getGreen(), m_color.getBlue());
  }


  public void setRainbow(){
     
    
    for(int i=0; i<m_ledBuffer.getLength(); i++){
      final int hue = (m_firstLight+(i*180/m_ledBuffer.getLength()))%180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
      
    }
    m_led.setData(m_ledBuffer);
    m_firstLight += 1;
    m_firstLight %= 180;
  }

  @Override
  public void periodic( ) {
    // This method will be called once per scheduler run
  }
}
