// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  /** ask Dan about compatiblity to mukanum drive*/
  private final CANSparkMax leftFront;
  private final CANSparkMax rightFront;
  private final CANSparkMax leftRear;
  private final CANSparkMax rightRear;
  private final MecanumDrive driveMecanum;


  public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_driveController = new XboxController(0);

  private final Drivetrain m_drivetrain = new Drivetrain();

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
  public ExampleSubsystem() {
    leftFront = new CANSparkMax(1, MotorType.kBrushless);
    rightFront = new CANSparkMax(1, MotorType.kBrushless);
    leftRear = new CANSparkMax(1, MotorType.kBrushless);
    rightRear = new CANSparkMax(1, MotorType.kBrushless);

    leftFront.restoreFactoryDefults();
    rightFront.restoreFactoryDefults();
    leftRear.restoreFactoryDefults();
    rightRear.restoreFactoryDefults();
    
  }
  MecanumDrive(leftFront, leftBack, rightFront, rightBack)


 /** Drives the robot using ArcadeDrive style control */
  public void arcadeDrive(double throttle, double rotation) {
    driveMecanum.driveCartesian(throttle, 0, rotation);
    driveGyro
  }

  /** Mecanum drive */ 
  public void mecanumDrive(double throttle, double slide, double rotation) { 

  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public class TeleopDrive extends CommandBase {
  /** Creates a new TeleopDrive. */
  }
  public TeleopDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public class TeleopDrive extends CommandBase {

  private final Drivetrain drivetrain;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
  }

   // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive the robot using driveCartesian style control
    // NOTE: The +/- for the y axis on the joystick is inverted from
    // the right-hand-rule
    drivetrain.driveCartesian(
        -throttle.getAsDouble(),
        rotation.getAsDouble());
  }
 
 public class TeleopDrive extends CommandBase {

  private final Drivetrain drivetrain;
  private final DoubleSupplier throttle;
  private final DoubleSupplier rotation;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(Drivetrain drivetrain, DoubleSupplier throttle, DoubleSupplier rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.throttle = throttle;
    this.rotation = rotation;
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive the robot using ArcadeDrive style control
    // NOTE: The +/- for the y axis on the joystick is inverted from
    // the right-hand-rule
    drivetrain.arcadeDrive(
        -throttle.getAsDouble(),
        rotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Set output to 0 for safety
    drivetrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
 }


}
