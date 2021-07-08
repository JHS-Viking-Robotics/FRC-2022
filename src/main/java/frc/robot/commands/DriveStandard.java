// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Arcade drive with standard percent output for Drivetrain subsystem */
public class DriveStandard extends CommandBase {

  private final Drivetrain drivetrain;
  private final DoubleSupplier throttle;
  private final DoubleSupplier rotation;

  /**
   * Creates a new DriveStandard.
   *
   * @param subsystem The drivetrain subsystem used by this command.
   */
  public DriveStandard(Drivetrain subsystem, DoubleSupplier throttle, DoubleSupplier rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    // NOTE: 'this' is used here when collecting parameter values, but not throughout rest of code
    // NOTE: Controller inputs are passed as DoubleSupplier class, and need to be converted to
    //       double whenever utilized. See WPI FRC docs on command-based examples for more info
    this.drivetrain = subsystem;
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
    // Drive with percent output from joystick
    drivetrain.arcadeDrivePercentOutput(throttle.getAsDouble(), rotation.getAsDouble());
    drivetrain.arcadeDrivePercentOutput(throttle.getAsDouble(), rotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Set output to 0 for safety
    drivetrain.arcadeDrivePercentOutput(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
