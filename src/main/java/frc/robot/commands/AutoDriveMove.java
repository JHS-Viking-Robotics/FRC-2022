// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoDriveMove extends CommandBase {
  Drivetrain drive;      // Drivetrain for robot
  Translation2d goal;    // Goal point for the robot
  Translation2d current; // Current point of the robot

  /** Creates a new AutoDriveMove. Moves the driveSubsystem to a position (x,y)
   * away from the start position according to a Translation2d reference frame.
   * This means +x is forward, +y is left */
  public AutoDriveMove(Drivetrain driveSubsystem, double x, double y) {
    this.drive = driveSubsystem;
    drive.resetOdometryMec();
    this.goal = new Translation2d(x, y);
    this.current = new Translation2d(0, 0);
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.mecanumDrive(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    current = drive.getPoseMec().getTranslation();
    drive.moveTo(goal.minus(current));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.mecanumDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (0.05 > goal.getDistance(current));
  }
}
