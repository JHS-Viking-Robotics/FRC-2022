// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain driveSystem;
  private final DoubleSupplier throttle;
  private final DoubleSupplier slide;
  private final DoubleSupplier rotation;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Drivetrain subsystem, DoubleSupplier throttle, DoubleSupplier slide, DoubleSupplier rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.driveSystem = subsystem;
    this.throttle = throttle;
    this.slide = slide;
    this.rotation = rotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSystem.mecanumDriveFOD(-throttle.getAsDouble(), slide.getAsDouble(), rotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSystem.mecanumDriveFOD(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
