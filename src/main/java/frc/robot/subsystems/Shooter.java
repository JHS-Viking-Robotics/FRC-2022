package frc.robot.subsystems;

import static frc.robot.Constants.Subsystem.Shooter.*;

import com.revrobotics.CANSparkMax; 
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  /** ask Dan about compatiblity to mukanum drive*/
  private final CANSparkMax front;
  private final CANSparkMax rear;

  private final DoubleSolenoid ShooterPCM;
  private boolean motorsOn = false;              // Current state of the motors

  public  Shooter() {
    front = new CANSparkMax(FRONT_ID, kBrushless);
    rear = new CANSparkMax(REAR_ID, kBrushless);
    ShooterPCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    front.restoreFactoryDefaults();
    rear.restoreFactoryDefaults();

    front.setInverted(FRONT_INVERTED);
    rear.setInverted(REAR_INVERTED);
  }

  //gssrhrthdfbhtrhsefgr
  public void togglePiston(boolean offOn){
    if (offOn == true){
      ShooterPCM.set(kForward);
    }
    else{
      ShooterPCM.set(kReverse);
    }
  }

  //turn on motors
  public void toggleMotors(){
    motorsOn = !motorsOn;
  }

  public void toggleTrigger(){
    ShooterPCM.toggle();
  }

  public void resetEncoder(){
    front.getEncoder().setPosition(0);
    rear.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    double output = (motorsOn) ? 0 : 0.5;
    front.set(output);
    rear.set(output);
  }
}
