package frc.robot.subsystems;

import frc.robot.subsystems.Shooter;
import static frc.robot.Constants.Subsystem.Intake.*;
import com.revrobotics.SparkMaxRelativeEncoder;

import com.revrobotics.CANSparkMax; 
import com.revrobotics.RelativeEncoder; 
import com.revrobotics.SparkMaxRelativeEncoder; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  /** Creates a new Shooter. */
  /** ask Dan about compatiblity to mukanum drive*/
  private final CANSparkMax takeMotor;

  private final RelativeEncoder takeMotorEncoder;

  public  Intake() {
    takeMotor = new CANSparkMax(TAKE_ID, MotorType.kBrushless);
    takeMotor.restoreFactoryDefaults();
    takeMotor.setInverted(TAKE_INVERTED);
    takeMotorEncoder = takeMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 22); 
  }
  public void toggleInTake(){
  double output =(takeMotorEncoder.getVelocity()>1) ? 0: 0.5;
  takeMotor.set(output);
  
}
public void resetEncoder(){
    takeMotor.getEncoder().setPosition(0);
  }
}

