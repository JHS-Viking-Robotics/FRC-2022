// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SetCurrentPosition extends CommandBase {

  private final Drivetrain drive;      // Drive subsystem to use
  private final Pose2d pose;           // Pose to reset the robot to

  /** Creates a new SetCurrentPosition command, which resets the Drivetrain
   * odometry to the specified point on the field. */
  public SetCurrentPosition(Drivetrain driveSubsystem, Pose2d pose) {
    this.drive = driveSubsystem;
    this.pose = pose;
    addRequirements(drive);
  }

  /** Creates a new SetCurrentPosition command, which resets the Drivetrain
   * odometry to the specified point on the field. */
  public SetCurrentPosition(Drivetrain driveSubsystem) {
    this(driveSubsystem, new Pose2d(0.381, 0.381, new Rotation2d()));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.drive(0, 0, 0, false);
    drive.resetOdometry(pose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
