package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;
public class AutoOffShoot extends CommandBase {
  private final Timer timer = new Timer();
  private final Shooter shooter;
  public AutoOffShoot (Shooter shooter){
    this.shooter = shooter;
    addRequirements(this.shooter);
  }
  @Override
  public void initialize() {
    timer.reset(); 
    timer.start(); 
    shooter.toggleMotors();
  }
  @Override
  public void execute() {
    
  }
   @Override
  public void end(boolean interrupted) {
    shooter.toggleMotors();
  }
  @Override
  public boolean isFinished() {
    return timer.get() > 5;
  }
}