package frc.robot.subsystems;

import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax; 
import com.revrobotics.RelativeEncoder; 
import com.revrobotics.SparkMaxRelativeEncoder; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  /** ask Dan about compatiblity to mukanum drive*/
  private final CANSparkMax topFront;
  private final CANSparkMax bottomFront;
  private final CANSparkMax topRear;
  private final CANSparkMax bottomRear;

  private final RelativeEncoder topFrontEncoder; // Left side front encoder 
  private final RelativeEncoder topRearEncoder; // Left side rear encoder 
  private final RelativeEncoder bottomFrontEncoder; // Right side front encoder 
  private final RelativeEncoder bottomRearEncoder; // Right side rear encoder
  private final DoubleSolenoid ShooterPCM;

  public  Shooter() {
    topFront = new CANSparkMax(Constants.Subsystem.Shooter.TOP_FRONT_ID, MotorType.kBrushless);
    bottomFront = new CANSparkMax(Constants.Subsystem.Shooter.BOTTOM_FRONT_ID, MotorType.kBrushless);
    topRear = new CANSparkMax(Constants.Subsystem.Shooter.TOP_BACK_ID, MotorType.kBrushless);
    bottomRear = new CANSparkMax(Constants.Subsystem.Shooter.BOTTOM_BACK_ID, MotorType.kBrushless);
    ShooterPCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    topFront.restoreFactoryDefaults();
    bottomFront.restoreFactoryDefaults();
    topRear.restoreFactoryDefaults();
    bottomRear.restoreFactoryDefaults();
     
    topFrontEncoder = topFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42); 
    topRearEncoder = topRear.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42); 
    bottomFrontEncoder = bottomFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42); 
    bottomRearEncoder = bottomRear.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
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
  bottomFront.set(output);
  bottomRear.set(output);
}
public void toggleTrigger(){
  ShooterPCM.toggle();
}
  
public void resetEncoder(){
    topFront.getEncoder().setPosition(0);
    bottomFront.getEncoder().setPosition(0);
    topRear.getEncoder().setPosition(0);
    bottomRear.getEncoder().setPosition(0);
  }
}