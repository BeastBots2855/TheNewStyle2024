// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class PhotonVision {
    //creates camera objects for fetching results from cameras
    private static PhotonCamera m_NoteTracker = new PhotonCamera("NoteDetector");
    private static PhotonCamera m_AprilTagTracker = new PhotonCamera("ApriltagTracker");
    private static PhotonPoseEstimator m_visionPoseEstimator;
    private static boolean m_AprilTagIsInSight;
    private static double[] lastBestNote = new double[]{0, 0};

  public PhotonVision(){
    try {
      m_visionPoseEstimator = new PhotonPoseEstimator(
        AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile), 
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
        m_AprilTagTracker, 
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6), 
                -Units.inchesToMeters(14), 
                Units.inchesToMeters(19.5)), 
            new Rotation3d(0, 0,0)));

    } catch(IOException e){
      System.out.println(e.getMessage() + "\n april tags didnt load");
    }
        
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


    public static PhotonPoseEstimator getPoseEstimator(){
        return m_visionPoseEstimator;
    }
}
