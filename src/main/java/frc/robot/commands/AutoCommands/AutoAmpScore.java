// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.MechanismSequences.SetIntakeInShooterAmp;
import frc.robot.commands.ShooterCommands.NewShooterRescind;
import frc.robot.subsystems.NewShooter;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.subsystems.WristFunctionality.IntakeWrist;
import frc.robot.subsystems.WristFunctionality.ShooterWrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAmpScore extends SequentialCommandGroup {
  /** Creates a new AutoAmpScore. */
  public AutoAmpScore(DriveSubsystem m_robotDrive, ShooterWrist m_ShooterWrist, IntakeWrist m_IntakeWrist, NewShooter Shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Since we are using a holonomic drivetrain, the rotation component of this pose
  Pose2d targetPose = DriverStation.getAlliance().get() ==  DriverStation.Alliance.Red ? 
    FieldConstants.RED_AMP_Robot :
    FieldConstants.BLUE_AMP_Robot;

// Create the constraints to use while pathfinding
PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
);
    addCommands(
      pathfindingCommand.alongWith(
        new SetIntakeInShooterAmp(m_ShooterWrist, m_IntakeWrist))
      .andThen(
        new NewShooterRescind(Shooter, ()-> 0.2))
      );
  }
}
