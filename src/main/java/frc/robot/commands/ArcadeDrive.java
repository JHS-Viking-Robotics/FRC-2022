// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private DoubleSupplier throttle;
  private DoubleSupplier rotation;
  
  /** Creates a new ArcadeDrive */
  public ArcadeDrive(Drivetrain drivetrain, DoubleSupplier throttle, DoubleSupplier rotation) {
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
    // Drive the robot using driveCartesian style control
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
