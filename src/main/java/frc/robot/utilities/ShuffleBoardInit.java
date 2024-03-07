// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WristFunctionality.IntakeWrist;
import frc.robot.subsystems.WristFunctionality.ShooterWrist;

/** Add your docs here. */
public class ShuffleBoardInit {

    private ShuffleboardTab m_telopOutput = Shuffleboard.getTab("Teleop");
    private ShuffleboardTab m_visionTab = Shuffleboard.getTab("Vision");

    public ShuffleBoardInit(
        XboxController m_driverController, XboxController m_operatorController, 
        DriveSubsystem m_robotDrive, Intake m_Intake, Shooter m_Shooter, 
        IntakeWrist m_IntakeWrist, ShooterWrist m_ShooterWrist, Indexer m_Indexer, 
        Climb m_Climb) {

            m_telopOutput.addDouble("Box anggle: ", () -> m_ShooterWrist.getAbsoluteEncoderValue());
            m_telopOutput.addDouble("Box setPoint: ", () -> m_ShooterWrist.getSetpoint());
            m_telopOutput.addBoolean("Box isWithinTolerance: ", () -> m_ShooterWrist.isWithinPidTolerance());
            m_telopOutput.addDouble("Intake anggle: ", () -> m_IntakeWrist.getAbsoluteEncoderValue());
            m_telopOutput.addBoolean("IntakeButtonIsPressed", ()-> m_Intake.isTouchingLimitSwitch());
            m_telopOutput.addBoolean("IndexerSwitchPressed", ()-> m_Indexer.isTouchingLimitSwitch());
            m_telopOutput.addBoolean("IndexerIsPrimed", ()-> m_Indexer.getIsPrimed());
            m_telopOutput.addDouble("ClimbAngle", m_robotDrive::getPitch);
            m_telopOutput.addBoolean("isIntakeWristPidOn", m_IntakeWrist::isPidEnabled);
            m_telopOutput.addBoolean("isShooterWristPidOn", m_ShooterWrist::isPidEnabled);

            m_visionTab.addDouble("DistanceFromRing", ()-> PhotonVision.getNotePidResponseVariable());
            m_visionTab.addDouble("RingX", ()-> PhotonVision.getConvertedLastNotePosition()[0]);
            m_visionTab.addDouble("RingY", ()-> PhotonVision.getConvertedLastNotePosition()[1]);

            SmartDashboard.putData(m_robotDrive);
            SmartDashboard.putData(m_IntakeWrist);
            SmartDashboard.putData(m_ShooterWrist);
            SmartDashboard.putData(m_Intake);
            SmartDashboard.putData(m_Shooter);
        }
}
