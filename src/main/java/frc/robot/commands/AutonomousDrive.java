package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import com.revrobotics.RelativeEncoder; 

public class AutonomousDrive extends CommandBase {
  private Drivetrain drivetrain;
  private DoubleSupplier throttle;
  private DoubleSupplier rotation;

  public AutonomousDrive(Drivetrain drivetrain, DoubleSupplier throttle, DoubleSupplier rotation) {
    this.drivetrain = drivetrain;
    this.throttle = throttle;
    this.rotation = rotation;
    addRequirements(this.drivetrain);
  }
  @Override
  public void initialize() {}
  @Override
  public void execute() {
    drivetrain.AutonomousDrive(
        -throttle.getAsDouble(),
        rotation.getAsDouble());
  }
  @Override
  public void end(boolean interrupted) {
    // Set output to 0 for safety
    drivetrain.arcadeDrive(0.0, 0.0);
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}