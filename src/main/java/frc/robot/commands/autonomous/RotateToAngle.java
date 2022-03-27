// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class RotateToAngle extends CommandBase {

  private final Drivetrain drive;       // Drive subsystem to use
  private final Rotation2d goal;        // Goal angle of the robot

  /** Creates a new RotateToAngle. */
  public RotateToAngle(Drivetrain driveSubsystem, Rotation2d goal, double maxSpeed) {
    this.drive = driveSubsystem;
    this.goal = goal;
    this.drive.setMaxSpeed(maxSpeed);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate how many degrees we still need to turn
    Rotation2d move = goal.minus(drive.getPose().getRotation());
    double rotation = (move.getRadians() > 0) ? 1.0 : -1.0;

    // We need to rotate our axis to use driveCartesian, which uses NED
    drive.drive(0.0, 0.0, -rotation, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Reset the max speed of the drivetrain to default and make sure we stopped
    drive.setMaxSpeed();
    drive.drive(0.0, 0.0, 0.0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // We won't get exactly there so stop within a few degrees
    return 5 > Math.abs(goal.minus(drive.getPose().getRotation()).getDegrees());
  }
}
