package frc.robot.subsystems;

import static frc.robot.Constants.Subsystem.Shooter.*;

import com.revrobotics.CANSparkMax; 
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Shooter extends SubsystemBase {
  private final CANSparkMax front;          // Front shooter motor
  private final CANSparkMax rear;           // Rear shooter motor
  private final DoubleSolenoid ShooterPCM;  // Solenoid for the shooter trigger
  
  private boolean motorsOn = false;         // Current state of the motors
  
  /** Creates a new Shooter. */
  public  Shooter() {
    front = new CANSparkMax(FRONT_ID, kBrushless);
    rear = new CANSparkMax(REAR_ID, kBrushless);
    ShooterPCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    front.restoreFactoryDefaults();
    rear.restoreFactoryDefaults();

    front.setInverted(FRONT_INVERTED);
    rear.setInverted(REAR_INVERTED);
  }

  /* Toggle the trigger on/off */
  public void togglePiston(boolean offOn){
    if (offOn == true){
      ShooterPCM.set(kForward);
    }
    else{
      ShooterPCM.set(kReverse);
    }
  }

  /* Toggle the spin-up motors on/off */
  public void toggleMotors(){
    motorsOn = !motorsOn;
  }

  /* Toggle the trigger on/off */
  public void toggleTrigger(){
    ShooterPCM.toggle();
  }

  @Override
  public void periodic() {
    double output = (motorsOn) ? 0.5 : 0;
    front.set(output);
    rear.set(output);
  }
}
