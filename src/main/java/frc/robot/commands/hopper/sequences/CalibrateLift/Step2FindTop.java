// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper.sequences.CalibrateLift;

import frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Step2FindTop extends CommandBase {

  private final Hopper hopper;
  
  /** Creates a new Step2FindTop. */
  public Step2FindTop(Hopper subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = subsystem;
    addRequirements(this.hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // The Lift.BOTTOM position should now be set, and the approximate range of
    // motion is about 890 ticks. Therefore the Lift should have around 2.5
    // seconds to reach 600 ticks. If this is a problem, step 1.5 can be made
    // which waits for the Lift to get to 850, then we can run this step
    hopper.setLift(600);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Increment the Lift up until the error is more than double our increment
    hopper.setLift(
        hopper.getLiftSetpoint() + 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Reset the Lift up setpoint to the new position on the sensor
    if (!interrupted) {
      hopper.resetLiftSetpoint(
        Hopper.Lift.UP,
        hopper.getLiftSetpoint());
    }
    hopper.setAllNeutral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Keep going up until the error gets higher than our increment
    return hopper.getLiftPositionError() > 4;
  }
}
