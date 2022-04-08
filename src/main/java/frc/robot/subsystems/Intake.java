package frc.robot.subsystems;

import static frc.robot.Constants.Subsystem.Intake.*;
import static edu.wpi.first.wpilibj.PneumaticsModuleType.*;

import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends SubsystemBase {

  private final CANSparkMax takeMotor; // Intake motor
  private final Solenoid intakeDrop;   // Piston holding up intake at start of match

  /** Creates a new .Intake */
  public  Intake() {
    takeMotor = new CANSparkMax(TAKE_ID, MotorType.kBrushless);
    takeMotor.restoreFactoryDefaults();
    takeMotor.setInverted(TAKE_INVERTED);

    // Piston should stay engaged, when it shuts off intake mechanism will fall
    // to position
    intakeDrop = new Solenoid(CTREPCM, INTAKE_DROP_ID);
    intakeDrop.set(true);
  }

  /** Turn the intake on */
  public void runIntake() {
    takeMotor.set(TAKE_SPEED);
  }

  /** Drop the intake to the normal position */
  public void toggleDrop(){
    intakeDrop.set(false);
  }
}
