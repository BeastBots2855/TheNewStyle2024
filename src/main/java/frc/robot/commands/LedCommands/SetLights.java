 package frc.robot.commands.LedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Colors;
import frc.robot.subsystems.LED;
import frc.robot.utilities.RGBColor;
 public class SetLights extends Command {
     private final LED m_ledString;
    private RGBColor m_color;

     public SetLights(LED subsystem, RGBColor m_color){
      m_ledString = subsystem;
      addRequirements(subsystem);
        this.m_color=m_color;
     }


     @Override
     public void initialize(){

     }
    
     @Override
     public void execute(){
      
        m_ledString.setColor(m_color);

        //  if(m_color.equals(Color.RED)){
        //    m_ledString.setColor(255,0,0); 
        //  }else if(m_color.equals(Color.BLUE)){
        //    m_ledString.setColor(0,0,255);              
        // }else if(m_color.equals(Color.YELLOW)){
        //     m_ledString.setColor(255,100,0);
        //  }else if(m_color.equals(Color.PURPLE)){
        //   m_ledString.setColor(150,0,150);
        //  }
      
        }

    @Override 
     public void end(boolean inturrupted) {

     }

     @Override
     public boolean isFinished(){
         return false;
    }
}
