package frc.robot.utilities;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FeedForwardCharacterization.FeedForwardCharacterizationData;

public class Autos {

    //FIXME: just rewrite all of this terribleness
    private final DriveSubsystem m_drivetrainSubsystem;
    private final Shooter m_Shooter;
    private SendableChooser<String> autoChooser;
    private HashMap<String, Command> m_commandMap;
   
    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    public Autos(DriveSubsystem drivetrainSubsystem, Shooter shooter){
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_Shooter = shooter;
        autoChooser = new SendableChooser<>();
        m_commandMap = new HashMap<>();
        
        // autoChooser.addOption(null, null);
        autoChooser.addOption("Drivetrain Characterization", "DrivetrainCharacterization");
        // autoChooser.addOption("Shooter Characterization", "ShooterCharacterization");
        // autoChooser.addOption("MoveBack", "MoveBack");
        m_commandMap.put("DrivetrainCharacterization", 
            new FeedForwardCharacterization(m_drivetrainSubsystem, true, new FeedForwardCharacterizationData("DriveSubsystem"), 
            m_drivetrainSubsystem::runCharacterizationVolts, m_drivetrainSubsystem::getCharacterizationVelocity));
        
            // m_commandMap.put("ShooterCharacterization", List.of(new FeedForwardCharacterization(drivetrainSubsystem, true, new FeedForwardCharacterizationData("Shooter"),
            // m_Shooter::runCharacterizationVolts , m_Shooter::getCharacterizationVelocity)));
        
        m_commandMap.put("Basic Test Auto", AutoBuilder.buildAuto("MoveBack"));
        
        // SmartDashboard.putData(autoChooser);
        autoTab.add(autoChooser);
        
    }
 

    public Command getAutoCommand() {
        String auto = autoChooser.getSelected();
        return m_commandMap.get(auto);
    }



}