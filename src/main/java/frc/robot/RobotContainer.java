// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_driveController = new XboxController(0);

  private final Drivetrain m_drivetrain = new Drivetrain();

  private final ArcadeDrive m_arcadeDrive
      = new ArcadeDrive(
          m_drivetrain,
          () -> m_driveController.getY(Hand.kLeft),
          () -> m_driveController.getX(Hand.kLeft));
  private final MecanumDrive m_mecanumDrive
      = new ArcadeDrive(
          m_drivetrain,
          () -> m_driveController.getY(Hand.kLeft),
          () -> m_driveController.getX(Hand.kLeft),
          () -> m_driveController.getX(Hand.kRight));
  private final MecanumDriveFOD m_mecanumDriveFOD
      = new ArcadeDrive(
          m_drivetrain,
          () -> m_driveController.getY(Hand.kLeft),
          () -> m_driveController.getX(Hand.kLeft),
          () -> m_driveController.getX(Hand.kRight));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set arcade drive as default
    m_drivetrain.setDefaultCommand(m_arcadeDrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      new JoystickButton(m_driveController, Button.kA.value)
          .onPress(m_mecanumDrive);
      new JoystickButton(m_driveController, Button.kB.value)
          .onPress(m_mecanumDriveFOD);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
