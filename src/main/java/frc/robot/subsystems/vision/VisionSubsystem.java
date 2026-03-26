// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public PhotonCamera hopperCamera = new PhotonCamera("8303_Cam_1_OV9281");
  public PhotonCamera frontCamera = new PhotonCamera("8303_Cam_2_OV9281");
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonPoseEstimator PoseEstimator;
  private double yaw;

  public VisionSubsystem() {}

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
              this.yaw = targetYaw;
            }
          }
        }
        SmartDashboard.putBoolean("Target Visible", targetVisible);
        SmartDashboard.putNumber("Target Yaw", this.yaw);
      }
  }
  
  public double getTargetYaw() {
    return this.yaw;
  }
}
