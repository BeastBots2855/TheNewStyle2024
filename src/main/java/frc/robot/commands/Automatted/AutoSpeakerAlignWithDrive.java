// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automatted;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AutoShoot;
import frc.robot.commands.Vision.SpeakerLockOn;
import frc.robot.commands.WristCommands.ShooterWristClosedLoop;
import frc.robot.commands.WristCommands.ShooterWristClosedLoopTracking;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.subsystems.WristFunctionality.ShooterWrist;
import frc.robot.utilities.PhotonVision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSpeakerAlignWithDrive extends ParallelCommandGroup {
  /** Creates a new AutoAimWithShooterAngle. */
  public AutoSpeakerAlignWithDrive(DriveSubsystem m_DriveSubsystem, ShooterWrist m_ShooterWrist, DoubleSupplier xDriveSupplier, DoubleSupplier yDriveSupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SpeakerLockOn(m_DriveSubsystem, xDriveSupplier, yDriveSupplier),
    new ShooterWristClosedLoopTracking(m_ShooterWrist, ()->AutoShoot.DISTANCE_TO_ANGLE_MAP.get(PhotonVision.getDistanceToSpeaker())));
  }
}
