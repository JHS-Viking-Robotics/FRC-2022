package frc.robot.subsystems;

import static frc.robot.Constants.Subsystem.Shooter.*;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static edu.wpi.first.wpilibj.PneumaticsModuleType.*;

import com.revrobotics.CANSparkMax; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Shooter extends SubsystemBase {
  private final CANSparkMax front;          // Front shooter motor
  private final CANSparkMax rear;           // Rear shooter motor
  private final DoubleSolenoid shooterPCM;  // Solenoid for the shooter trigger
  
  private boolean motorsOn = false;         // Current state of the motors
  
  /** Creates a new Shooter. */
  public  Shooter() {
    front = new CANSparkMax(FRONT_ID, kBrushless);
    rear = new CANSparkMax(REAR_ID, kBrushless);
    shooterPCM = new DoubleSolenoid(CTREPCM, TRIGGER_FORWARD_ID, TRIGGER_REVERSE_ID);

    front.restoreFactoryDefaults();
    rear.restoreFactoryDefaults();

    front.setInverted(FRONT_INVERTED);
    rear.setInverted(REAR_INVERTED);
  }

  /* Toggle the trigger on/off */
  public void togglePiston(boolean offOn){
    if (offOn == true){
      shooterPCM.set(kForward);
    }
    else{
      shooterPCM.set(kReverse);
    }
  }

  /* Toggle the spin-up motors on/off */
  public void toggleMotors(){
    motorsOn = !motorsOn;
  }

  /* Toggle the trigger on/off */
  public void toggleTrigger(){
    shooterPCM.toggle();
  }

  @Override
  public void periodic() {
    double output = (motorsOn) ? 0.5 : 0;
    front.set(output);
    rear.set(output);
  }
}
