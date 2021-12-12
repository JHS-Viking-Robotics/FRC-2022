// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper.sequences.Dispense;

import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.Lift;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Step1DispensePosition extends CommandBase {

  private final Hopper hopper;

  /** Creates a new Step1DispensePosition command. */
  public Step1DispensePosition(Hopper subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = subsystem;
    addRequirements(this.hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Increment the Lift down until the error is more than double our increment
    hopper.setLift(Lift.DISPENSE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Wait until we get to the dispense position
    return Math.abs(hopper.getLiftPositionError()) < 50;
  }
}
