// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.Subsystem;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.MecanumDriveFOD;
import frc.robot.commands.MecanumDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Shooter m_shooter = new Shooter();

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
          .whenPressed(m_mecanumDrive);
      new JoystickButton(m_driveController, Button.kB.value)
          .whenPressed(m_mecanumDriveFOD);
      new JoystickButton(m_driveController, Button.kY.value)
          .whenPressed(new InstantCommand(m_shooter::toggleMotors, m_shooter));
      new JoystickButton(m_driveController, Button.kX.value)
          .whenPressed(new InstantCommand(m_shooter::toggleTrigger, m_shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create voltage constraints and trajectory configuration
    DifferentialDriveVoltageConstraint voltConstraint
        = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Subsystem.Drivetrain.S,
                    Subsystem.Drivetrain.V,
                    Subsystem.Drivetrain.A),
                Subsystem.Drivetrain.DIFF_KINEMATICS,
                10);
    TrajectoryConfig config = new TrajectoryConfig(
        Subsystem.Drivetrain.MAX_VELOCITY,
        Subsystem.Drivetrain.MAX_ACCELERATION);
    config.setKinematics(Subsystem.Drivetrain.DIFF_KINEMATICS)
        .addConstraint(voltConstraint);

    // Generate the trajectory. This basic placeholder follows the Ramsette
    // controller guide in the WPILib docs, and makes an S shape which ends
    // 3 meters forward
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
        List.of(
            new Translation2d(1.0, 1.0),
            new Translation2d(-2.0, 1.0)),
        new Pose2d(3.0, 0.0, new Rotation2d(0.0)),
        config);

    // Generate the Ramsette Controller Command. To use an imported Pathweaver
    // Trajectory in json format, replace trajectory with autonTrajectory below
    RamseteCommand command = new RamseteCommand(
        trajectory,
        m_drivetrain::getPoseDiff,
        new RamseteController(),
        new SimpleMotorFeedforward(
            Subsystem.Drivetrain.S,
            Subsystem.Drivetrain.V,
            Subsystem.Drivetrain.A),
        Subsystem.Drivetrain.DIFF_KINEMATICS,
        m_drivetrain::getWheelSpeedsDiff,
        new PIDController(Subsystem.Drivetrain.P, 0, 0),
        new PIDController(Subsystem.Drivetrain.P, 0, 0),
        m_drivetrain::tankDriveVoltage,
        m_drivetrain);

    // Reset the odometry to the initial Pose2d of the trajectory
    m_drivetrain.resetOdometryDiff(trajectory.getInitialPose());

    // Run the command, and stop at the end
    return command.andThen(() -> m_drivetrain.tankDriveVoltage(0.0, 0.0));
  }
}
