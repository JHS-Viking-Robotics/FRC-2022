// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CollectBalls extends CommandBase {

  private final Hopper hopper;

  /**
   * Creates a new CollectBalls.
   * 
   * @param subsystem The {@link Hopper} subsystem used by this command
   */
  public CollectBalls(Hopper subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = subsystem;
    addRequirements(this.hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Put the Lift on the floor on start
    hopper.setLift(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run the Intake in continuously
    hopper.setIntake(Hopper.Intake.IN);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop running the Intake and return the Lift to rest position
    hopper.setIntake(Hopper.Intake.NEUTRAL);
    hopper.setLift(800);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
