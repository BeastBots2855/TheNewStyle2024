// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoShoot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Vision;

/** Add your docs here. */
public class PhotonVision extends SubsystemBase{
    //creates camera objects for fetching results from cameras
    private static PhotonCamera m_NoteTracker = new PhotonCamera("NoteDetector");
    private static PhotonCamera m_AprilTagTracker = new PhotonCamera("AprilTagTracker");
    private static PhotonPoseEstimator m_visionPoseEstimator;
    private static AprilTagFieldLayout fieldLayout;
    private static double[] lastBestNote = new double[]{0, 0};
    private static Timer m_Timer = new Timer();
    private static Field2d m_photonVisionField = new Field2d();

    private static double displacementToTargetAngle = 0;
    private static double displacementToSpeakerX = 0;
    private static double displacementToSpeakery = 0;

  public PhotonVision(){
    try {
      m_visionPoseEstimator = new PhotonPoseEstimator(
        AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile), 
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
        m_AprilTagTracker, 
        new Transform3d(
            new Translation3d(
                -Units.inchesToMeters(13), 
                Units.inchesToMeters(7.5), 
                Units.inchesToMeters(22.5)), 
            new Rotation3d(Math.PI / 2, -0.349 , Math.PI)));
    } catch(IOException e){
      System.out.println(e.getMessage() + "\n vision estimator initialization failed");
    }
    m_Timer.start();
    
    fieldLayout = Vision.kTagLayout;
    SmartDashboard.putData("directVision", m_photonVisionField);
        
  } 
    
    /**
     * calculates a response variable that can be used to lock the heading of the robot to a ring
     * no matter the orientation of the camera. Note that this does require calibration of an 
     * arbitrarty coordinate system and two points in the field of view of the camera. One point A
     * that is located at the center of the robot along the front edge of the robot and some point B
     * that is perpindicular to the front edge of the robot
     * @return an interable PID loop measurment variable
     */
    public static double getNotePidResponseVariable(){
        calculateBestNote();
        return NoteLocalization.getSignedDistanceFromNearestPathToNote(lastBestNote); 
    }

    /**
     * looks through all viable targets in frame and uses the calculateBestTarget method to find
     * the best target within the cameras view. 
     */
    public static void calculateBestNote(){
        var results = m_NoteTracker.getLatestResult();
        if(results.hasTargets()){
            var NotesFromPhotonVision = results.getTargets();
            double[][] Notes = new double[NotesFromPhotonVision.size()][2];
            for(int i = 0; i < NotesFromPhotonVision.size(); i++){
                Notes[i] = new double[]{NotesFromPhotonVision.get(i).getYaw(), NotesFromPhotonVision.get(i).getPitch()};
            }
            lastBestNote = NoteLocalization.calculateBestTarget(Notes);
        }
    }

    /**
     * gets the position of note within the relative coordinate system
     */
    public static double[] getConvertedLastNotePosition() {
        return new double[]{lastBestNote[0], lastBestNote[1]};
    }

    public static boolean canTrustNoteData(){
        return m_Timer.get() < 0.5;
    }

    public static PhotonPoseEstimator getPoseEstimator(){
        return m_visionPoseEstimator;
    }

    @Override
    public void periodic(){
    var results = m_NoteTracker.getLatestResult();
    if(results.hasTargets()){
         m_Timer.reset();
    }
    }

     public static void addFilteredPoseData(Pose2d currentPose, SwerveDrivePoseEstimator m_poseEstimator) {
            PhotonPoseEstimator poseEstimator = PhotonVision.getPoseEstimator();
                // print out the time for this line to run 
                Optional<EstimatedRobotPose> pose = poseEstimator.update();
                if (pose.isPresent()) {
                    Pose3d pose3d = pose.get().estimatedPose;
                    Pose2d pose2d = pose3d.toPose2d();
                    if (
                        pose3d.getX() >= -FieldConstants.VISION_FIELD_MARGIN &&
                        pose3d.getX() <= FieldConstants.FIELD_LENGTH + FieldConstants.VISION_FIELD_MARGIN &&
                        pose3d.getY() >= -FieldConstants.VISION_FIELD_MARGIN &&
                        pose3d.getY() <= FieldConstants.FIELD_WIDTH + FieldConstants.VISION_FIELD_MARGIN &&
                        pose3d.getZ() >= -FieldConstants.VISION_Z_MARGIN &&
                        pose3d.getZ() <= FieldConstants.VISION_Z_MARGIN
                    ) {
                        double sum = 0.0;
                        for (PhotonTrackedTarget target : pose.get().targetsUsed) {
                            Optional<Pose3d> tagPose =
                                fieldLayout.getTagPose(target.getFiducialId());
                            if (tagPose.isEmpty()) continue;
                            sum += currentPose.getTranslation().getDistance(tagPose.get().getTranslation().toTranslation2d());
                        }

                        int tagCount = pose.get().targetsUsed.size();
                        double stdScale = Math.pow(sum / tagCount, 2.0) / tagCount;
                        double xyStd = FieldConstants.VISION_STD_XY_SCALE * stdScale;
                        double rotStd = FieldConstants.VISION_STD_ROT_SCALE * stdScale;
                        //time this as well
                        m_poseEstimator.addVisionMeasurement(pose2d, pose.get().timestampSeconds, VecBuilder.fill(xyStd, xyStd, rotStd));
                    }

                    m_photonVisionField.setRobotPose(pose2d);
                }
            }

        public static Translation2d getGoalPose(){
            boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            .equals(DriverStation.Alliance.Blue);
            Translation2d goalPose = isBlue ? FieldConstants.BLUE_SPEAKER : FieldConstants.RED_SPEAKER;
            return goalPose;
        }
        
        public static Translation2d getAdjustedSpeakerPosition(Pose2d currentPose, ChassisSpeeds robotVel) {
            Translation2d goalPose = getGoalPose();
            double distanceToSpeaker = currentPose.getTranslation().getDistance(goalPose);
            double x = goalPose.getX()
                    - (robotVel.vxMetersPerSecond * (distanceToSpeaker / FieldConstants.NOTE_VELOCITY));
            double y = goalPose.getY() - (robotVel.vyMetersPerSecond * (distanceToSpeaker / FieldConstants.NOTE_VELOCITY));
            Translation2d goalPoseAdjusted = new Translation2d(x, y);
            // Pose2d speaker = new Pose2d(goalPoseAdjusted, new Rotation2d());
            // m_goalPoseField.setRobotPose(speaker);
            return goalPoseAdjusted; 
        }

        public static double getShooterAngle(Pose2d currentPose, ChassisSpeeds robotVel){
            double distance = currentPose.getTranslation().getDistance(getAdjustedSpeakerPosition(currentPose, robotVel));
            return AutoShoot.DISTANCE_TO_ANGLE_MAP.get(distance);
        }

        public static double getTagetAngleRobotToSpeaker(Pose2d currentPose, ChassisSpeeds robotVel) {
            double x = getAdjustedSpeakerPosition(currentPose, robotVel).getX() - currentPose.getX();
            displacementToSpeakerX = x;
            double y = getAdjustedSpeakerPosition(currentPose, robotVel).getY() - currentPose.getY();
            displacementToSpeakery = y;
            // System.out.println(Math.atan2(y, x));
            return Math.atan2(y, x);
        }

        public static double getRobotToSpeakerAngleXDisplacement() {
            return displacementToSpeakerX;
        }

        public static double getRobotToSpeakerAngleYDisplacement() {
            return displacementToSpeakery;
        }

        public static void setDisplacementToTargetAngle(double displacement){
            displacementToTargetAngle = displacement;
        }

        public static double getDisplacementToTargetAngle(){
            return displacementToTargetAngle;
        }

        
}
