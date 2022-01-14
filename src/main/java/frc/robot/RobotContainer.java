// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.drivetrain.*;
import frc.robot.commands.hopper.*;
import frc.robot.commands.hopper.sequences.Dispense.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Define robot subsystems, commands, input devices, and buttons
  private final Drivetrain m_drivetrain;
  private final Hopper m_hopper;
  private final DriveStandard m_driveStandard;
  private final CollectBalls m_hopperCollectBalls;
  private final Step1DispensePosition m_hopperDispensePosition;
  private final Step2Unload m_hopperDispenseUnload;
  private final CalibrateLift m_hopperCalibrate;
  private final Manual m_hopperManual;
  private final TestMode m_hopperTestMode;
  private final XboxController m_driveController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Instantiate robot subsystems, commands, input devices, and buttons
    m_drivetrain = new Drivetrain();
    m_hopper = new Hopper();
    m_driveController = new XboxController(Constants.Joystick.DRIVER);
    m_driveStandard = new DriveStandard(
        m_drivetrain,
        () -> m_driveController.getY(Hand.kLeft),
        () -> m_driveController.getX(Hand.kLeft));
    m_hopperCollectBalls = new CollectBalls(m_hopper);
    m_hopperDispensePosition = new Step1DispensePosition(m_hopper);
    m_hopperDispenseUnload = new Step2Unload(m_hopper);
    m_hopperCalibrate = new CalibrateLift(m_hopper);
    m_hopperManual = new Manual(
        m_hopper,
        () -> m_driveController.getBumper(Hand.kLeft),
        () -> m_driveController.getBumper(Hand.kRight),
        () -> m_driveController.getY(Hand.kRight));
    m_hopperTestMode = new TestMode(m_hopper);

    // Configure the button bindings
    configureButtonBindings();

    // Set subsystem default commands
    m_drivetrain.setDefaultCommand(m_driveStandard);

    // Configure Shuffleboard
    configureShuffleboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driveController, Button.kY.value)
        .whenHeld(m_hopperDispensePosition);
    new JoystickButton(m_driveController, Button.kY.value)
        .whenReleased(m_hopperDispenseUnload);
    new JoystickButton(m_driveController, Button.kA.value)
        .whenHeld(m_hopperCollectBalls);
    new JoystickButton(m_driveController, Button.kStart.value)
        .whenPressed(m_hopperCalibrate, false);
  }

  /**
   * Configures the Shuffleboard default tab with all subsystems and basic
   * information, as well as the subsystem tabs with their command lists.
   * 
   * <p>This method should not be used for sending NetworkTables values specific
   * to subsystems or commands, but instead is for objects used in
   * RobotContainer that need to go on the Shuffleboard.
   */
  private void configureShuffleboard() {
    // Add subsystems and command scheduler to the default tab
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_hopper);

    // Add manual overrides and test mode toggles to the Dashboard
    SmartDashboard.putData("Hopper Manual Override", m_hopperManual);
    SmartDashboard.putData("Hopper Test Mode", m_hopperTestMode);
  }

  /**
   * Schedule any test commands. Should be called from {@link Robot#testInit()}
   * after clearing all active commands
   */
  public void enableTestMode() {
    m_hopperTestMode.schedule();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_driveStandard;
  }
}
