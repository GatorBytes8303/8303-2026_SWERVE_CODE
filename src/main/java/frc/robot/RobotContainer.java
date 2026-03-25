// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.SpitterSubsystem;
import frc.robot.subsystems.intake.SuckerSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.commands.drive.VisionAimCommand;
import frc.robot.commands.intake.SpitterForwardCommand;
import frc.robot.commands.intake.SpitterRetractCommand;
import frc.robot.commands.intake.SuckerShootCommand;
import frc.robot.commands.intake.SuckerSlowCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final SuckerSubsystem m_sucker = new SuckerSubsystem();
  public final SpitterSubsystem m_spitter = new SpitterSubsystem();
  public final VisionSubsystem m_vision = new VisionSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Zero the heading of the robot when the start button is pressed.
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    // Spit from intake if jamed on A pressed
        new JoystickButton(m_driverController, XboxController.Button.kA.value)
    .whileTrue(new SpitterRetractCommand(m_spitter));  
    
    // Sucker and spitter controls
    // Right bumper shoots spitter and sucker, left bumper retracts spitter, Y button kicks into spitter and sucks slow
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
    .whileTrue(new SuckerShootCommand(m_sucker));
    
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
    .whileTrue(new SpitterForwardCommand(m_spitter));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
    .whileTrue(new SpitterRetractCommand(m_spitter));

        new JoystickButton(m_driverController, XboxController.Button.kY.value)
    .whileTrue(new SpitterRetractCommand(m_spitter));

        new JoystickButton(m_driverController, XboxController.Button.kY.value)
    .whileTrue(new SuckerSlowCommand(m_sucker));

    // Test Vision aim at target when X is held
      new JoystickButton(m_driverController, XboxController.Button.kX.value)
    .whileTrue(new VisionAimCommand(m_robotDrive, m_vision));
}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return  null;
  }
}