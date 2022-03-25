// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.MecanumDrive;
import frc.robot.commands.FireBall;
import frc.robot.commands.autonomous.GetOffLine;
import frc.robot.subsystems.*;

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
  private final XboxController m_liftController = new XboxController(1);

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Shooter m_shooter = new Shooter();
  private final Lift m_lift = new Lift();
  private final Intake m_intake= new Intake();

  private final MecanumDrive m_mecanumDrive
      = new MecanumDrive(
          m_drivetrain,
          () -> m_driveController.getLeftY(),
          () -> m_driveController.getLeftX(),
          () -> m_driveController.getRightX(),
          false);
  private final MecanumDrive m_mecanumDriveFOD
      = new MecanumDrive(
          m_drivetrain,
          () -> m_driveController.getLeftY(),
          () -> m_driveController.getLeftX(),
          () -> m_driveController.getRightX(),
          true);
  private final FireBall m_fireBall
      = new FireBall(
        m_shooter);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set arcade drive as default, and also set lift.stop as a safety
    m_drivetrain.setDefaultCommand(m_mecanumDrive);
    m_lift.setDefaultCommand(new RunCommand(m_lift::stop, m_lift));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driveController, Button.kX.value)
        .whenPressed(new InstantCommand(m_shooter::toggleMotors, m_shooter));
    new JoystickButton(m_driveController, Button.kY.value)
        .whenPressed(m_fireBall);

    new JoystickButton(m_liftController, Button.kY.value)
        .whenHeld(new RunCommand(m_lift::goUp, m_lift));
    new JoystickButton(m_liftController, Button.kA.value)
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
    return new GetOffLine(m_drivetrain, 0.4, true);
  }
}