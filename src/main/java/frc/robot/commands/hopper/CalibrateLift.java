// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CalibrateLift extends CommandBase {

  private final Hopper hopper;

  /** Creates a new Step1FindBottom. */
  public CalibrateLift(Hopper subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = subsystem;
    addRequirements(this.hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.resetLiftSensorPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Increment the Lift down until the error is high
    hopper.setLift(
        hopper.getLiftSetpoint() - 10);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Reset the Lift sensor 
    hopper.resetLiftSensorPosition();
    hopper.setLift(Hopper.Lift.DOWN);
    hopper.setAllNeutral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Keep going down until the error gets higher than our increment
    return hopper.getLiftPositionError() > 100;
  }
}
