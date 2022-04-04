package frc.robot.subsystems;

import static frc.robot.Constants.Subsystem.Intake.*;

import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.PneumaticsModuleType.*;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends SubsystemBase {

  private final CANSparkMax takeMotor; // Intake motor
  private boolean intakeOn = false;    // State of intake motors
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
  public void toggleInTake() {
    double output =(intakeOn) ? 0 : TAKE_SPEED;
    intakeOn = !intakeOn;
    takeMotor.set(output);
  }
  public void toggleDrop(){
   intakeDrop.set(false);
  }
}
