// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Manual extends CommandBase {

  private final Hopper hopper;
  private final DoubleSupplier liftSpeed;
  private final BooleanSupplier intakeInButton;
  private final BooleanSupplier intakeOutButton;

  /**
   * Creates a new CollectBalls.
   * 
   * @param subsystem The {@link Hopper} subsystem used by this command
   */
  public Manual(Hopper subsystem, BooleanSupplier intakeInButton, BooleanSupplier intakeOutButton, DoubleSupplier liftSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = subsystem;
    this.liftSpeed = liftSpeed;
    this.intakeInButton = intakeInButton;
    this.intakeOutButton = intakeOutButton;
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
    // Run the Intake and Lift
    hopper.setLift(liftSpeed.getAsDouble() * 0.5);
    if (intakeInButton.getAsBoolean()) {
      hopper.setIntake(Intake.IN);
    } else if (intakeOutButton.getAsBoolean()) {
      hopper.setIntake(Intake.OUT);
    } else {
      hopper.setIntake(Intake.NEUTRAL);
    }
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
