package frc.robot.subsystems;

import static frc.robot.Constants.Subsystem.Intake.*;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax; 
import com.revrobotics.RelativeEncoder; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  /** Creates a new Shooter. */
  /** ask Dan about compatiblity to mukanum drive*/
  private final CANSparkMax takeMotor;

  private final RelativeEncoder takeMotorEncoder;

  private boolean intakeOn = false;                  // State of intake motors

  public  Intake() {
    takeMotor = new CANSparkMax(TAKE_ID, MotorType.kBrushless);
    takeMotor.restoreFactoryDefaults();
    takeMotor.setInverted(TAKE_INVERTED);
    takeMotorEncoder = takeMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42); 
  }

  public void toggleInTake() {
    double output =(intakeOn) ? 0: 0.5;
    intakeOn = !intakeOn;
    takeMotor.set(output);
  }

  public void resetEncoder() {
    takeMotor.getEncoder().setPosition(0);
  }
}
