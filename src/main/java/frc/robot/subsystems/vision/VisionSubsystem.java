// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

import frc.robot.Constants.VisionConstants;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public PhotonCamera hopperCamera = new PhotonCamera(VisionConstants.kHopperCameraName);
  public PhotonCamera frontCamera = new PhotonCamera(VisionConstants.kFuelCameraName);
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonPoseEstimator PoseEstimator;
  private double yaw;
  private double range;
  private boolean isColorPresented = false;
  private boolean isRed = false;

  public VisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Read in relevant data from the Camera
    // We will use the hopper camera to determine the alliance color and calculate yaw and range to the hub.
    boolean targetVisible = false;
    double targetYaw = 0.0;
    double targetRange = 0.0;
    var results = hopperCamera.getAllUnreadResults();

    // If the alliance color is not yet determined, attempt to determine it using the DriverStation API.
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent() && !isColorPresented) {
      if (ally.get() == Alliance.Red) {
        isColorPresented = true;
        isRed = true;
      }
      else if (ally.get() == Alliance.Blue) {
        isColorPresented = true;
        isRed = false;
      }
    }

      if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
          for (var target : result.getTargets()) {
            // Check if the target is the correct color's hub tag.
            if (isRed ? target.getFiducialId() == VisionConstants.kRedHubTagId : target.getFiducialId() == VisionConstants.kBlueHubTagId) {
              // This is the correct target, so we can use it to calculate yaw and range.
              targetYaw = target.getYaw();
              targetVisible = true;
              this.yaw = targetYaw;
              targetRange =
                PhotonUtils.calculateDistanceToTargetMeters(
                  VisionConstants.kHopperCameraHeightMeters, 
                  VisionConstants.kHubTagHeightMeters, 
                  Units.degreesToRadians(VisionConstants.kHopperCameraPitchDegrees), 
                  Units.degreesToRadians(target.getPitch()));
              this.range = targetRange;
            }
          }
        }
        // Publish data to SmartDashboard for debugging purposes.
        SmartDashboard.putString("Alliance", isRed ? "Red" : "Blue");
        SmartDashboard.putNumber("Target Tracking", isRed ? VisionConstants.kRedHubTagId : VisionConstants.kBlueHubTagId);
        SmartDashboard.putBoolean("Target Visible", targetVisible);
        SmartDashboard.putNumber("Target Yaw", this.yaw);
        SmartDashboard.putNumber("Target Range", this.range);
        SmartDashboard.putNumber("Desired Camera to Target Distance", getDesiredCameraToTargetDistance());
      }
  }
  
  public double getTargetYaw() {
    return this.yaw;
  }
  
  public double getTargetRange() {
    return this.range;
  }

  public double getDesiredCameraToTargetDistance() {
    return getTargetRange() - VisionConstants.kHubTagDistanceMeters;
  }
}
