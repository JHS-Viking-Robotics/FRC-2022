package frc.robot.subsystems;

import static frc.robot.Constants.Subsystem.Intake.*;

import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {

  private final CANSparkMax takeMotor; // Intake motor
  private boolean intakeOn = false;    // State of intake motors

  /** Creates a new Shooter. */
  public  Intake() {
    takeMotor = new CANSparkMax(TAKE_ID, MotorType.kBrushless);
    takeMotor.restoreFactoryDefaults();
    takeMotor.setInverted(TAKE_INVERTED);
  }

  /* Turn the intake on/off */
  public void toggleInTake() {
    double output =(intakeOn) ? 0 : TAKE_SPEED;
    intakeOn = !intakeOn;
    takeMotor.set(output);
  }
}
