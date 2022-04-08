// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class GetOffLine extends CommandBase {

  private final Drivetrain drive; // Drivetrain subsystem to use
  private final boolean forward;  // Whether we are moving forward/backwards off line

  /**
   * Creates a new GetOffLine command
   * 
   * @param driveSubsystem the Drivetrain to use
   * @param speed speed to move at (0.0, 1.0]
   * @param forward whether to go forward or backwards off the line
  */
  public GetOffLine(Drivetrain driveSubsystem, double speed, boolean forward) {
    this.drive = driveSubsystem;
    this.forward = forward;
    drive.setMaxSpeed(speed);
    drive.setBrake();
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.drive(0, 0, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = forward ? 1.0 : -1.0;
    drive.drive(throttle, 0.0, 0.0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop moving and reset drive speed
    drive.drive(0, 0, 0, false);
    drive.setCoast();
    drive.setMaxSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Go at least 1m in either direction, plus some wiggle room
    return forward ? drive.getDistanceLeft() > 2.0 : drive.getDistanceLeft() < -2.0;
  }
}
