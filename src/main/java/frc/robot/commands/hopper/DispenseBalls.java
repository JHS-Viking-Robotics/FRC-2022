// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.Intake;
import frc.robot.subsystems.Hopper.Lift;

public class DispenseBalls extends CommandBase {

  private final Hopper hopper;
  private final Timer timer;

  /**
   * Creates a new DispenseBalls.
   *
   * @param subsystem The {@link Hopper} subsystem used by this command
   */
  public DispenseBalls(Hopper subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = subsystem;
    timer = new Timer();
    addRequirements(this.hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Put the Hopper in the dispense position
    hopper.setLift(Lift.UP);

    // Start the timer
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopper.setIntake(Intake.OUT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.setIntake(Intake.NEUTRAL);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Finished after dispensing for 3 seconds
    return (timer.get() >= 3.0);
  }
}
