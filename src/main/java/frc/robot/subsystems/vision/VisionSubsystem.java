// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public PhotonCamera hopperCamera = new PhotonCamera("Hopper Camera");
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonPoseEstimator PoseEstimator;

  public VisionSubsystem() {
    PoseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, hopperCamera, VisionConstants.RobotToCam);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        // Read in relevant data from the Camera
    boolean targetVisible = false;
    double targetYaw = 0.0;
    var results = hopperCamera.getAllUnreadResults();

      if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
          for (var target : result.getTargets()) {
            if (target.getFiducialId() == VisionConstants.kHopperTagId) {
              // Found Tag 35, record its information
              targetYaw = target.getYaw();
              targetVisible = true;
            }
          }
        }
      }
        // Put debug information to the dashboard
        SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
        SmartDashboard.putNumber("Vision Target Yaw", targetYaw);
  }

  public PhotonPipelineResult getAllUnreadResults(){
    return hopperCamera.getAllUnreadResults();
  }

  public Optional<EstimatedRobotPose> getVisionPoseEstimationResult(PhotonPipelineResult results){
    return PoseEstimator.update(results);
  }

  public EstimatedRobotPose ifExistsGetEstimatedRobotPose(){
    if (getVisionPoseEstimationResult().isPresent()){
      return getVisionPoseEstimationResult().get();
    } 
    return null;
  }

  public PhotonTrackedTarget getBestTarget(){
    return getLatestResult().getBestTarget();
  }

  public Transform3d getCamToTarget(){
    return getBestTarget().getBestCameraToTarget();
  }

  //Returns list of IDs currently being tracked
  public List<Integer> getAprilTagIDs(){
    List<PhotonTrackedTarget> targets = getLatestResult().getTargets();
    List<Integer> tagIDs = new ArrayList<>();
    targets.forEach(target -> tagIDs.add(target.getFiducialId()));

    return tagIDs;
  }

  //Returns true if an the ID is being tracked
  public boolean containsID(Integer ID){
    return getAprilTagIDs().contains(ID);
  }

}
