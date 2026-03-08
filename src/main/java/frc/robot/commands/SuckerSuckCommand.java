// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.SuckerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SuckerSuckCommand extends Command {
  private final SuckerSubsystem m_sucker;

  /**
   * Runs the sucker
   *
   * @param sucker The subsystem used by this command.
   */
  public SuckerSuckCommand(SuckerSubsystem sucker) {
    m_sucker = sucker;
    addRequirements(sucker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_sucker.runSucker(ScorerConstants.kSuckerMotorSpeed * -1);
  }

  // Called once the command ends or is interrupted. Here we ensure the sucker is not
  // running once we let go of the button
  @Override
  public void end(boolean interrupted) {
    m_sucker.runSucker(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}