package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveSubsystem;

public class Autos {

    //FIXME: just rewrite all of this terribleness
    private final DriveSubsystem m_drivetrainSubsystem;
    private final Shooter m_Shooter;
    private SendableChooser<String> autoChooser;
    private HashMap<String, List<Command>> m_commandMap;
   
    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    public Autos(DriveSubsystem drivetrainSubsystem, Shooter shooter){
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_Shooter = shooter;
        autoChooser = new SendableChooser<>();
        m_commandMap = new HashMap<>();
        
        autoChooser.addOption("Drivetrain Characterization", "DrivetrainCharacterization");
        autoChooser.addOption("Shooter Characterization", "ShooterCharacterization");
        autoChooser.addOption("Basic Test Auto", "Basic Test Auto");
        autoChooser.addOption("4Ring", "4Ring");
        m_commandMap.put("DrivetrainCharacterization", List.of(
            new FeedForwardCharacterization(m_drivetrainSubsystem, true, new FeedForwardCharacterizationData("DriveSubsystem"), 
            m_drivetrainSubsystem::runCharacterizationVolts, m_drivetrainSubsystem::getCharacterizationVelocity),
            new FeedForwardCharacterization(m_drivetrainSubsystem, true, new FeedForwardCharacterizationData("DriveSubsystem"), 
            m_drivetrainSubsystem::runCharacterizationVolts, m_drivetrainSubsystem::getCharacterizationVelocity)));
        
            // m_commandMap.put("ShooterCharacterization", List.of(new FeedForwardCharacterization(drivetrainSubsystem, true, new FeedForwardCharacterizationData("Shooter"),
            // m_Shooter::runCharacterizationVolts , m_Shooter::getCharacterizationVelocity)));
        
        m_commandMap.put("Basic Test Auto", List.of(AutoBuilder.buildAuto("Basic Test Auto")));
        m_commandMap.put("4Ring", List.of(AutoBuilder.buildAuto("4Ring")));
        
        // SmartDashboard.putData(autoChooser);
        autoTab.add(autoChooser);
        
    }
 

    public Command getAutonomousCommand() {
        String auto = autoChooser.getSelected();
        return m_commandMap.get(auto).get(0);
    }
}