// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class MoveToLocation extends CommandBase {

  private final Drivetrain drive;       // Drive subsystem to use
  private final Translation2d goal;     // Goal position of the robot
  private final Translation2d start;    // Start position of the robot

  /**
   * Creates a new MoveToLocation.
   * 
   * @param driveSubsystem Drivetrain to use
   * @param goal Goal position of the robot relative to the field, units of meters
   * @param maxSpeed Max speed of the robot during the movement, between (0, 1]
   */
  public MoveToLocation(Drivetrain driveSubsystem, Translation2d goal, double maxSpeed) {
    this.drive = driveSubsystem;
    this.goal = goal;
    this.start = drive.getPose().getTranslation();
    this.drive.setMaxSpeed(maxSpeed);
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // We should start out not moving
    drive.drive(0, 0, 0, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate a unit vector representing our desired translation
    Translation2d move = goal.minus(drive.getPose().getTranslation());
    move = move.div(move.getNorm());

    // Scale our vector so we speed up and slow down in the last 0.5m
    // If we are only moving 1m total, then just go slowly

    // We need to rotate our axis to use driveCartesian, which uses NED
    drive.drive(
      move.getX(),
      -move.getY(),
      0.0,
      true);
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
    return goal.equals(drive.getPose().getTranslation());
  }
}
