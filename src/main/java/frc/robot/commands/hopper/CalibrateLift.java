// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Hopper;
import frc.robot.commands.hopper.sequences.CalibrateLift.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CalibrateLift extends SequentialCommandGroup {
  /** Creates a new Sequence. */
  public CalibrateLift(Hopper subsystem) {
    addCommands(
        new Step1FindBottom(subsystem),
        new Step2FindTop(subsystem));
  }
}
