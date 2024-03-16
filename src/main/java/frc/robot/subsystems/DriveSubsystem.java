// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

//stuff
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Vision;
import frc.robot.utilities.PhotonVision;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
  //private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);


  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
  //     DriveConstants.kDriveKinematics,
  //     getHeadingAsRotation2D(),
  //     new SwerveModulePosition[] {
  //         m_frontLeft.getPosition(),
  //         m_frontRight.getPosition(),
  //         m_rearLeft.getPosition(),
  //         m_rearRight.getPosition()
  //     });

    private final SwerveDrivePoseEstimator m_poseEstimator =
    new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getHeadingAsRotation2D(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        new Pose2d(0, 0, new Rotation2d(0)), 
        VecBuilder.fill(0.85, 0.85, Units.degreesToRadians(0.5)), // initiial was 0.05 for both on top and 0.5 for bottom, 0.05, 0.05, 0.65
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(60))); // 0.5, 0.5, 50 
      
    Field2d m_field = new Field2d();
    

  private boolean isCharacterizing = false;
  private double characterizationVolts = 0;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
      //System.out.println(m_poseEstimator.getEstimatedPosition());

      SmartDashboard.putData("field", m_field);
      //m_gyro.reset();
    
  }
  

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    // if(Vision.isVisionEnabled){
  //     // PhotonVision.getPoseEstimator().update().ifPresent(estimatedRobotPose ->
  //     // {
  //     //   m_poseEstimator.addVisionMeasurement(
  //     //     estimatedRobotPose.estimatedPose.toPose2d(), 
  //     //     estimatedRobotPose.timestampSeconds);
          
  //     // });
  //     m_poseEstimator.update(
  //       getHeadingAsRotation2D(),
  //       new SwerveModulePosition[] {
  //         m_frontLeft.getPosition(),
  //         m_frontRight.getPosition(),
  //         m_rearLeft.getPosition(),
  //         m_rearRight.getPosition()
  //       });
  // }else {
  //   m_odometry.update(
  //       getHeadingAsRotation2D(),
  //       new SwerveModulePosition[] {
  //           m_frontLeft.getPosition(),
  //           m_frontRight.getPosition(),
  //           m_rearLeft.getPosition(),
  //           m_rearRight.getPosition()
  //   });

  PhotonVision.addFilteredPoseData(getPose2d(), m_poseEstimator);
   m_poseEstimator.update(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
    m_field.setRobotPose(getPose2d());
   


      //   PhotonVision.getPoseEstimator().update().ifPresent(estimatedRobotPose ->
      // {
      //   m_poseEstimator.addVisionMeasurement(
      //     estimatedRobotPose.estimatedPose.toPose2d(), 
      //     estimatedRobotPose.timestampSeconds);
          
      // });
    
   
    //System.out.println(getPose2d());


    
  // }
    
   


    if(isCharacterizing){
      m_frontLeft.runCharacterization(characterizationVolts, DriveConstants.kFrontLeftChassisAngularOffset);
      m_frontRight.runCharacterization(characterizationVolts, DriveConstants.kFrontRightChassisAngularOffset);
      m_rearLeft.runCharacterization(characterizationVolts, DriveConstants.kBackLeftChassisAngularOffset);
      m_rearRight.runCharacterization(characterizationVolts, DriveConstants.kBackRightChassisAngularOffset);

      
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */


  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
      m_poseEstimator.resetPosition(getHeadingAsRotation2D(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        }, 
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed * 0.5;
    double dt = 0.02;
    // var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
    //     fieldRelative
    //         ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-m_gyro.getAngle()))
    //         : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    var swerveModuleStates =
      DriveConstants.kDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds.discretize(
              fieldRelative 
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getPose2d().getRotation()) 
              : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered),
              dt));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }



  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
//     boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
//     .equals(DriverStation.Alliance.Blue);
// Rotation2d heading = isBlue ? new Rotation2d() : new Rotation2d(Math.PI);

// m_poseEstimator.resetPosition(
//     Rotation2d.fromDegrees(getHeading()),
//     new SwerveModulePosition[] {
//             m_frontLeft.getPosition(),
//             m_frontRight.getPosition(),
//             m_rearLeft.getPosition(),
//             m_rearRight.getPosition()
//     },
//     new Pose2d(m_poseEstimator.getEstimatedPosition().getX(), m_poseEstimator.getEstimatedPosition().getY(),
//             heading));
    System.out.println("Reset Gyro");
  }

  /**
   * Returns the heading of the robot looped to increase past 360
   */
  public Rotation2d getHeadingAsRotation2D() {
    return Rotation2d.fromDegrees(getAngle());
  }

  public double getAngle(){
    return -m_gyro.getAngle();
  }

  public Pose2d getPose2d(){
      return m_poseEstimator.getEstimatedPosition();
  }

  public double getPitch() {
    return m_gyro.getPitch();
  }



  public MAXSwerveModule[] getMaxSwerveModules()
  {
    MAXSwerveModule[] maxArray = {
      m_frontLeft, 
      m_frontRight, 
      m_rearLeft, 
      m_rearRight};
    return maxArray;
  }


  public SwerveModuleState[] getModuleStates()
  {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    MAXSwerveModule[] modules = getMaxSwerveModules();
    for(int i = 0; i < modules.length; i++)
    {
       moduleStates[i] = modules[i].getState();
    }
    return moduleStates;
  }


  
  public ChassisSpeeds getChassisSpeeds()
  {
    return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

// it

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void runCharacterizationVolts(double volts){
    isCharacterizing = true;
    characterizationVolts = volts;
  }

  

  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    driveVelocityAverage += m_frontRight.getCharacterizationVelocity();
    driveVelocityAverage += m_rearRight.getCharacterizationVelocity();
    driveVelocityAverage += m_frontLeft.getCharacterizationVelocity();
    driveVelocityAverage += m_rearRight.getCharacterizationVelocity();
    return driveVelocityAverage / 4.0;
  
  }

  public void setRobotRelativeSpeeds(ChassisSpeeds chassisSpeeds)
  {
    //chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
  m_frontLeft.setDesiredState(swerveModuleStates[0]);
  m_frontRight.setDesiredState(swerveModuleStates[1]);
  m_rearLeft.setDesiredState(swerveModuleStates[2]);
  m_rearRight.setDesiredState(swerveModuleStates[3]);
  }



  public void configureAutoBuilder(){
      AutoBuilder.configureHolonomic(
      this::getPose2d,
      this::resetOdometry,
      this::getChassisSpeeds,
      this::setRobotRelativeSpeeds, 
       AutoConstants.autoBuilderPathConfig,
       () -> {var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;},
      this);
  }

}
