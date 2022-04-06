// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TripleBallBottomRight extends SequentialCommandGroup {
  /** Creates a new TripleBallBottomRight. */
  public TripleBallBottomRight(Drivetrain drive, Shooter shooter, Intake intake, double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Configure robot before starting
        new SetCurrentPosition(
            drive,
            new Pose2d(
                7.59,
                2.87,
                Rotation2d.fromDegrees(-111))),
        // Fire first ball
        new ShootSequence(shooter),
        // Move to get second ball
        new RotateToAngle(drive, Rotation2d.fromDegrees(-89), speed),
        new ScheduleCommand(new RunCommand(intake::run, intake)),
        new MoveToLocation(drive, new Translation2d(7.62, 0.65), speed),
        new InstantCommand(intake::stop, intake),
        // Fire second ball
        new RotateToAngle(drive, Rotation2d.fromDegrees(-110), speed),
        new MoveToLocation(drive, new Translation2d(7.84, 2.77), speed),
        new ShootSequence(shooter),
        // Move to get third ball
        new RotateToAngle(drive, Rotation2d.fromDegrees(-160), speed),
        new ScheduleCommand(new RunCommand(intake::run, intake)),
        new MoveToLocation(drive, new Translation2d(5.16, 1.93), speed),
        new InstantCommand(intake::stop, intake),
        // Fire third ball
        new RotateToAngle(drive, Rotation2d.fromDegrees(-110), speed),
        new MoveToLocation(drive, new Translation2d(7.84, 2.77), speed),
        new ShootSequence(shooter)
    );
  }
}
