package frc.robot.utilities;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OldShooter;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.utilities.FeedForwardCharacterization.FeedForwardCharacterizationData;


public class Autos {

    //FIXME: just rewrite all of this terribleness
    private final DriveSubsystem m_drivetrainSubsystem;
    private SendableChooser<String> autoChooser;
    private HashMap<String, Command> m_commandMap;
   
    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    public Autos(DriveSubsystem drivetrainSubsystem){
        m_drivetrainSubsystem = drivetrainSubsystem;
        autoChooser = new SendableChooser<>();
        m_commandMap = new HashMap<>();
        
        // autoChooser.addOption(null, null);
        //autoChooser.addOption("Drivetrain Characterization", "DrivetrainCharacterization");
        autoChooser.addOption("BlueFour", "BlueFour");
        autoChooser.addOption("Move", "Move");
        autoChooser.addOption("RedFour", "RedFour");
        autoChooser.addOption("AmpSideAuto", "AmpSideAuto");
        autoChooser.addOption("LazyAmp", "LazyAmp");
        autoChooser.addOption("OnePieceMobility", "OnePieceMobility");
        autoTab.add(autoChooser);
        
    }
 

    public Command getAutoCommand() {
        String auto = autoChooser.getSelected();
        return m_commandMap.get(auto);
    }

    public void mapCommands(){
        
        m_commandMap.put("FourNoteAuto", AutoBuilder.buildAuto("FourNoteAuto"));
        m_commandMap.put("ThreeNoteAuto", AutoBuilder.buildAuto("ThreeNoteAuto"));
        m_commandMap.put("FourFast", AutoBuilder.buildAuto("FourFast"));
        m_commandMap.put("FourFromThree", AutoBuilder.buildAuto("FourFromThree"));
        m_commandMap.put("AmpSideAuto", AutoBuilder.buildAuto("AmpSideAuto"));
        m_commandMap.put("LazyAmp", AutoBuilder.buildAuto("LazyAmp"));
        m_commandMap.put("OnePieceMobility", AutoBuilder.buildAuto("OnePieceMobility"));
        m_commandMap.put("Move", AutoBuilder.buildAuto("Move"));
    //     m_commandMap.put("DrivetrainCharacterization", 
    //         new FeedForwardCharacterization(m_drivetrainSubsystem, true, new FeedForwardCharacterizationData("DriveSubsystem"), 
    //         m_drivetrainSubsystem::runCharacterizationVolts, m_drivetrainSubsystem::getCharacterizationVelocity));
    }



}