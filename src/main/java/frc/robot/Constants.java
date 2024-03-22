// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.util.Random;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.RGBColor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final double kchassisRadiusMeters = Math.pow(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2), 0.5) * 0.0254;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 3;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 4;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId =2;
    public static final int kRearRightTurningCanId = 6;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kOperatorControllerPort = 1;
    public static final int kRumbleValue = 1;
  }

  public static final class IntakeWristConstants {
    public static final int IntakeWristCANID = 21;
  }

    public static final class IntakeConstants {
      public static final int IntakeMotorCANID = 20;
      public static final double motorEjectSpeed = -0.5;
  }

  public static final class ShooterWristConstants {
      public static final int ShooterWristCANID = 30;
  }

  public static final class ShooterConstants {
      public static final int ShooterMotorCANID = 31;
      public static final int ShooterMotor2CANID = 33;
      public static final double kMotorConsumeSpeed = 0.3;
  }

  public static final class IndexerConstants {
      public static final int IndexerMotorCANID = 32;
      public static final double kMotorIndexSpeed = 0.3;

  }


  

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;


      public static final HolonomicPathFollowerConfig autoBuilderPathConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
      new PIDConstants(8, 0.0 ,0), //original p = 5, 1st attempt: p = 5, d = 0.5, 2nd attempt: p= 5, d = 0.5, 3rd attempt: p = 5, d = 3 this caused the wheels to shutter
      new PIDConstants(5, 0.0, 0), //5.0, 0, 0.2
      DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
      DriveConstants.kchassisRadiusMeters, // Drive base radius in meters. Distance from robot center to furthest module.
      new ReplanningConfig());

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;

  }

  public static final class LimitSwitchConstants {
    public static final int kIntakeButton = 3;
    public static final int kIntakeWristBack = 8;
    public static final int kShooterWristBack = 9;
    public static final int kIndexerSwitch = 2;
  }

  public static final class ClimbConstants {
    public static final int kLeftTopCanID = 50;
    public static final int kLeftBottomCanID = 51;
    public static final int kRightTopCanID = 52;
    public static final int kRightBottomCanID = 53;
  }

  public static final class PIDSetPoint{
    // public static final double kIntakeGroundPickup = 3;
    // public static final double kIntakePassOff = 180;
    // public static final double kShooterPassOff = 140;
    // public static final double kShooterAmp = 45;
    // public static final double kShooterSpeaker = 131;
    // public static final double kShooterClimb = 90;

    public static final double kIntakeGroundPickup = 3;
    public static final double kIntakePassOff = 180;
    public static final double kShooterPassOff = 55;
    public static final double kShooterAmp = 330;
    public static final double kShooterSpeaker = 41;
    public static final double kShooterClimb = 90;

  }
public class Colors { 
    public static final RGBColor red = new RGBColor(255, 0, 0);
    public static final RGBColor blue = new RGBColor(0,0,255);
    public static final RGBColor green = new RGBColor(0,255,0);
    public static final RGBColor purple = new RGBColor(150, 0, 150);
    public static final RGBColor yellow = new RGBColor(255,100,0);
    public static final RGBColor cyan = new RGBColor(0,150,150);
    public static final RGBColor orange = new RGBColor(255,50,0);
  }

  


  public static final class Vision {

        public static final boolean isVisionEnabled = true;
        public static final String kAprilTagCameraName = "ApriltagTracker";
        public static final String kNoteTrackerCameraName = "NoteDetector";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToAprilTagCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
        public static final Transform3d kRobotToNoteCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));


        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
      
        public static final Vector<N3> odometryStd = VecBuilder.fill(0.06, 0.06, 0.01);
        public static final Vector<N3> visionStd = VecBuilder.fill(0.35, 0.35, 0.8);


        
    }

    public static class FieldConstants {
        public static final double VISION_FIELD_MARGIN = 0.5;
        public static final double VISION_Z_MARGIN = 0.75;
        public static final double VISION_STD_XY_SCALE = 0.02;
        public static final double VISION_STD_ROT_SCALE = 0.065;//0.035;

        public static final double FIELD_LENGTH = 16.5417;
        public static final double FIELD_WIDTH = 8.0136;

        public static final double NOTE_VELOCITY = 10.0;

        public static final Translation2d BLUE_SPEAKER = new Translation2d(0.0241, 5.547868);
        public static final Translation2d RED_SPEAKER = new Translation2d(FIELD_LENGTH - BLUE_SPEAKER.getX(),
            BLUE_SPEAKER.getY() + 0.1);
        public static final Translation2d STAGE = new Translation2d(4.981067, 4.105783);

        public static final double SPEAKER_HEIGHT = 2.08;
        public static final Pose3d BLUE_SPEAKER_3D = new Pose3d(BLUE_SPEAKER.getX(), BLUE_SPEAKER.getY(), SPEAKER_HEIGHT,
            new Rotation3d());
        public static final Pose3d RED_SPEAKER_3D = new Pose3d(RED_SPEAKER.getX(), RED_SPEAKER.getY(), SPEAKER_HEIGHT,
            new Rotation3d());

        public static final double OPPONENT_WING_LINE = 10.66;
        public static final double AMP_X = 1.9;

        public static final PIDConstants AUTO_AIM_ROT_PID_CONSTANTS = new PIDConstants(9.5, 0.01, 0.5);
        public static final PIDConstants AUTO_AIM_ROT_PID_CONSTANTS_TELE = new PIDConstants(10.5, 0.01, 0.5);

        public static final double VISION_REJECT_DISTANCE = 2.3;
    }

    public static class AutoShoot {
      public static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_MAP = new InterpolatingDoubleTreeMap();
    static {
      DISTANCE_TO_ANGLE_MAP.put(2.45, 44.9);
      DISTANCE_TO_ANGLE_MAP.put(1.51, 53.3);
      DISTANCE_TO_ANGLE_MAP.put(3.42, 36.5);
      // DISTANCE_TO_ANGLE_MAP.put(2.2, ArmConstants.kOffset - 0.077);
      // DISTANCE_TO_ANGLE_MAP.put(3.0, ArmConstants.kOffset - 0.059);
      // DISTANCE_TO_ANGLE_MAP.put(4.1, ArmConstants.kOffset - 0.044);
      // DISTANCE_TO_ANGLE_MAP.put(4.9, ArmConstants.kOffset - 0.035);
      // DISTANCE_TO_ANGLE_MAP.put(5.5, ArmConstants.kOffset - 0.029);
    }
    }



}

