// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/** Arcade drive with velocity controlled output for Drivetrain subsystem */
public class DriveVelocity extends CommandBase {
  
  private final Drivetrain drivetrain;
  private final DoubleSupplier throttle;
  private final DoubleSupplier rotation;
  
  /**
   * Creates a new DriveVelocity.
   * 
   * @param subsystem The drivetrain subsystem used by this command.
   * @param throttle The Supplier from a joystick for the throttle (through x axis)
   * @param rotation The Supplier from a joystick for the rotation (about z axis)
  */
  public DriveVelocity(Drivetrain subsystem, DoubleSupplier throttle, DoubleSupplier rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
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
    // Drive with velocity PID from joystick
    drivetrain.arcadeDriveVelocity(throttle.getAsDouble(), rotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDriveVelocity(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
