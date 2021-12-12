// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper.sequences.Dispense;

import frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Step2Unload extends CommandBase {

  private final Hopper hopper;
  private final Timer timer;

  /** Creates a new Step2Unload command. */
  public Step2Unload(Hopper subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = subsystem;
    this.timer = new Timer();
    addRequirements(this.hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.setLift(Hopper.Lift.DISPENSE);
    timer.reset();
    timer.start();
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopper.setIntake(Hopper.Intake.OUT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Wait until we get to four seconds
    return timer.get() >= 4.0;
  }
}
