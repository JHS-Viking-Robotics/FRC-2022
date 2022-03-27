// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;

public class InitializeIntake extends CommandBase {

  private final Lift lift;  // Lift subsystem

  /** Creates a new InitializeIntake. */
  public InitializeIntake(Lift lift) {
    this.lift = lift;
    addRequirements(this.lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lift.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lift.set(0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lift.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lift.getHeight() > 0.20;
  }
}
