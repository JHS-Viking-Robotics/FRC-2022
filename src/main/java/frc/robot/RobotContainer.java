// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.MecanumDriveFOD;
import frc.robot.commands.MecanumDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_driveController = new XboxController(0);
  private final XboxController m_shooterController = new XboxController(1);

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Shooter m_shooter = new Shooter();
  private final Lift m_lift = new Lift();
  private final Intake m_intake= new Intake();

  private final ArcadeDrive m_arcadeDrive
      = new ArcadeDrive(
          m_drivetrain,
          () -> m_driveController.getLeftY(),
          () -> m_driveController.getLeftX());
  private final MecanumDrive m_mecanumDrive
      = new MecanumDrive(
          m_drivetrain,
          () -> m_driveController.getLeftY(),
          () -> m_driveController.getLeftX(),
          () -> m_driveController.getRightX());
  private final MecanumDriveFOD m_mecanumDriveFOD
      = new MecanumDriveFOD(
          m_drivetrain,
          () -> m_driveController.getLeftY(),
          () -> m_driveController.getLeftX(),
          () -> m_driveController.getRightX());
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set arcade drive as default, and also set lift.stop as a safety
    m_drivetrain.setDefaultCommand(m_arcadeDrive);
    m_lift.setDefaultCommand(new RunCommand(m_lift::stop, m_lift));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driveController, Button.kA.value)
        .whenPressed(m_mecanumDrive);
    new JoystickButton(m_driveController, Button.kB.value)
        .whenPressed(m_mecanumDriveFOD);

    new JoystickButton(m_driveController, Button.kX.value)
        .whenPressed(new InstantCommand(m_shooter::toggleMotors, m_shooter));
    new JoystickButton(m_driveController, Button.kY.value)
        .whenPressed(new InstantCommand(m_shooter::toggleTrigger, m_shooter));

    new JoystickButton(m_shooterController, Button.kY.value)
        .whenHeld(new RunCommand(m_lift::goUp, m_lift));
    new JoystickButton(m_shooterController, Button.kA.value)
        .whenHeld(new RunCommand(m_lift::goDown, m_lift));

    new JoystickButton(m_driveController, Button.kRightBumper.value)
        .whenPressed(new InstantCommand(m_intake::toggleInTake, m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_arcadeDrive;
  }
}