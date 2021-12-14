// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.Intake;
import frc.robot.subsystems.Hopper.Lift;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestMode extends CommandBase {

  private final Hopper hopper;

  /**
   * Creates a new TestMode command.
   * 
   * @param subsystem The {@link Hopper} subsystem used by this command
   */
  public TestMode(Hopper subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = subsystem;
    addRequirements(this.hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Zero out the motors at start
    hopper.setAllNeutral();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Testing mode automatically grabs the desired output from the Dashboard
    hopper.setLift(Lift.TESTING);
    hopper.setIntake(Intake.TESTING);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Set output to 0 for safety
    hopper.setAllNeutral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
