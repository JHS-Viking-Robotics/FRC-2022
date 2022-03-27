// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.InitializeIntake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoubleShootRight extends SequentialCommandGroup {
  /** Creates a new DoubleShootRight. */
  public DoubleShootRight(Drivetrain drivetrain, Shooter shooter, Lift lift,
                          Intake intake, double speed, boolean forward) {
    // Order goes as such:
    //     1. Let the Robot know where we start (use some reference or line on
    //        the field so it's easy to line up during matches. Angle must be
    //        VERY ACCURATE!)
    //     2. Set the shooter speed to its auton setting (so it shoots low port.
    //        We are having issues with this :/ )
    //     3. Add your steps in order. Rotation and translation are seperate.
    //        Make sure you leave enough time to spin up
    //     4. Use ShootSequence to automatically fire a ball (takes about 3s,
    //        handles motors and piston for you. It doesnt call setMotorSpeed
    //        so you still need that once at the beginning and end of your path)
    //     5. Remember that the intake is on the front and shooter is on the
    //        back so make sure you rotate the right way!
    addCommands(
        // Configuration before starting
        new SetCurrentPosition(
            drivetrain,
            new Pose2d(
                7.65,
                0.83,
                Rotation2d.fromDegrees(2))),
        new InstantCommand(shooter::setMotorSpeedAuton, shooter),
        // Go to Waypoint 1
        new RotateToAngle(drivetrain, Rotation2d.fromDegrees(-112.37), speed),
        new MoveToLocation(drivetrain, new Translation2d(7.65, 2.69), speed),
        // Fire first ball
        new ShootSequence(shooter),
        new InitializeIntake(lift),
        // Head to Waypoint 3 with intake running to get the ball
        new RotateToAngle(drivetrain, Rotation2d.fromDegrees(-96.38), speed),
        new InstantCommand(intake::toggleInTake, intake),
        new MoveToLocation(drivetrain, new Translation2d(7.65, 0.63), speed),
        new InstantCommand(intake::toggleInTake, intake),
        // Head to final waypoint to score last ball
        new RotateToAngle(drivetrain, Rotation2d.fromDegrees(-112.37), speed),
        new MoveToLocation(drivetrain, new Translation2d(7.79, 2.65), speed),
        new ShootSequence(shooter),
        // Clean up when we are done
        new InstantCommand(shooter::setMotorSpeed, shooter)
    );
  }
}
