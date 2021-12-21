// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import frc.robot.commands.hopper.sequences.Dispense.*;
import frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Dispense extends SequentialCommandGroup {

  /**
   * Creates a new Dispense sequence command.
   *
   * @param subsystem The {@link Hopper} subsystem used by this command
   */
  public Dispense(Hopper subsystem) {
    addCommands(
        new Step1DispensePosition(subsystem),
        new Step2Unload(subsystem));
  }
}
