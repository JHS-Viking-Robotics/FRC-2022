package frc.robot.subsystems;

import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax; 
import com.revrobotics.RelativeEncoder; 
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  /** ask Dan about compatiblity to mukanum drive*/
  private final CANSparkMax topFront;
  private final CANSparkMax topRear;

  private final RelativeEncoder topFrontEncoder; // Left side front encoder 
  private final RelativeEncoder topRearEncoder; // Left side rear encoder 
  private final DoubleSolenoid ShooterPCM;

  public  Shooter() {
    topFront = new CANSparkMax(Constants.Subsystem.Shooter.TOP_FRONT_ID, kBrushless);
    topRear = new CANSparkMax(Constants.Subsystem.Shooter.TOP_BACK_ID, kBrushless);
    ShooterPCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    topFront.restoreFactoryDefaults();
    topRear.restoreFactoryDefaults();
     
    topFrontEncoder = topFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42); 
    topRearEncoder = topRear.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42); 
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
    double output =(topFrontEncoder.getVelocity()>1) ? 0: 0.5;
    topFront.set(output);
    topRear.set(output);
  }

  public void toggleTrigger(){
    ShooterPCM.toggle();
  }
  
public void resetEncoder(){
    topFront.getEncoder().setPosition(0);
    topRear.getEncoder().setPosition(0);
  }
}
