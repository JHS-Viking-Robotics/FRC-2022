package frc.robot.subsystems;

import static frc.robot.Constants.Subsystem.Intake.*;

import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.PneumaticsModuleType.*;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends SubsystemBase {

  private final CANSparkMax takeMotor; // Intake motor
  private final Solenoid intakeDrop ;

  /** Creates a new .Intake */
  public  Intake() {
    takeMotor = new CANSparkMax(TAKE_ID, MotorType.kBrushless);
     intakeDrop = new Solenoid(CTREPCM, INTAKE_DROP_ID);
    //We what it to stay up untill it gets changed
    intakeDrop.set(true);
    takeMotor.restoreFactoryDefaults();
    takeMotor.setInverted(TAKE_INVERTED);
  }

  /* Turn the intake on/off */
  public void run() {
    takeMotor.set(TAKE_SPEED);
  }

  /* Drop the intake to the normal position */
  public void toggleDrop(){
   intakeDrop.set(false);
  }

  /* Turn the intake off */
  public void stop() {
    takeMotor.set(0.0);
  }
}
