// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAndScoot extends SequentialCommandGroup {

  /** Creates a new ShootAndScoot. */
  public ShootAndScoot(Drivetrain drivetrain, Shooter shooter, double speed, boolean forward) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    shooter.setMotorSpeed(1.0);
    addCommands(
      new WaitCommand(7.5),
      new InstantCommand(shooter::toggleMotors, shooter),
      new WaitCommand(2.5),
      new InstantCommand(shooter::toggleTrigger, shooter),
      new WaitCommand(2.0),
      new GetOffLine(drivetrain, speed, forward)
    );
    shooter.setMotorSpeed();
  }
}
