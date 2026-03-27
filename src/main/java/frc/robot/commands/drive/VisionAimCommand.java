// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.Constants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionAimCommand extends Command {
  // Vision subsystem used to obtain target yaw and range values.
  private final VisionSubsystem m_vision;

  // Drive subsystem used to send drive/aiming setpoints.
  private final DriveSubsystem m_robotDrive;

  /**
   * Create a new VisionAimCommand.
   *
   * @param vision The VisionSubsystem that provides target yaw and range.
   * @param drive The DriveSubsystem used to control the robot's drivetrain.
   */
  public VisionAimCommand(VisionSubsystem vision, DriveSubsystem drive) {
    // Save references to required subsystems.
    m_vision = vision;
    m_robotDrive = drive;

    // Declare subsystem requirements so the scheduler won't run conflicting
    // commands that also require the drive or vision subsystems.
    addRequirements(drive);
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* 
    Directly drive using the latest measurements from the vision system.
    
    Sets distance to the target as the error between the current range and the desired range 
    (the distance from the camera to the tag, if positive the camera is too far so move forward).
    
    Sets the angle to the target as the error between the current yaw and the desired yaw 
    (0, or directly facing the target).
    */
    m_robotDrive.drive(
        m_vision.getTargetRange() - VisionConstants.kHubTagDistanceMeters,
        0,
        m_vision.getTargetYaw(),
        DriveConstants.kFieldOrientedDrive);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
