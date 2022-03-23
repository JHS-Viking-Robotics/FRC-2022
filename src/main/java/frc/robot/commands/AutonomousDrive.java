package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutonomousDrive extends CommandBase {
  private Drivetrain drivetrain;
  private DoubleSupplier throttle;
  private DoubleSupplier rotation;
}
