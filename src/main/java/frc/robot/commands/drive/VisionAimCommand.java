// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionAimCommand extends Command {
  private final VisionSubsystem m_vision;
  private final DriveSubsystem m_robotDrive;
  /** Creates a new VisionAim. */
  public VisionAimCommand(VisionSubsystem vision, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = vision;
    m_robotDrive = drive;
    addRequirements(drive);
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        new RunCommand(
            () -> m_robotDrive.drive(
                0.0,
                m_vision.getTargetYaw(),
                0.0,
                false));
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
