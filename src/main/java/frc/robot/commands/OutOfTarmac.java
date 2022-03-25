package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class OutOfTarmac extends CommandBase {

  private final Drivetrain drive; 
  private final boolean forward;
  
  public OutOfTarmac(Drivetrain driveSubsystem, double speed, boolean forward) {
    this.drive = driveSubsystem;
    this.forward = forward;
    drive.setMaxSpeed(speed);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.drive(0, 0, 0, false);
  }

  @Override
  public void execute() {
    double throttle = forward ? 1.0 : -1.0;
    drive.drive(throttle, 0.0, 0.0, false);
  }

  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0, false);
    drive.setMaxSpeed();
  }
  
  @Override
  public boolean isFinished() {
    return forward ? drive.getDistanceLeft() > 1.1 : drive.getDistanceLeft() < -1.1;
  }
}