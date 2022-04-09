package frc.robot.subsystems;

import java.util.Map;

import static frc.robot.Constants.Subsystem.Shooter.*;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static edu.wpi.first.wpilibj.PneumaticsModuleType.*;
import static edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets.*;

import com.revrobotics.CANSparkMax; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Shooter extends SubsystemBase {
  private final CANSparkMax front;          // Front shooter motor
  private final CANSparkMax rear;           // Rear shooter motor
  private final DoubleSolenoid shooterPCM;  // Solenoid for the shooter trigger
  private NetworkTableEntry shooterSpeed;   // NetworkTables controller for motor speed
  private boolean motorsOn = false;         // Current state of the motors
  
  /** Creates a new Shooter. */
  public  Shooter() {
    front = new CANSparkMax(FRONT_ID, kBrushless);
    rear = new CANSparkMax(REAR_ID, kBrushless);
    shooterPCM = new DoubleSolenoid(CTREPCM, TRIGGER_FORWARD_ID, TRIGGER_REVERSE_ID);
    //has to be set before toggled
    toggleTrigger(false);

    front.restoreFactoryDefaults();
    rear.restoreFactoryDefaults();

    front.setInverted(FRONT_INVERTED);
    rear.setInverted(REAR_INVERTED);

    // Configure the shuffleboard so we can easily test the Shooter
    configureShuffleboard();
  }

  /** Configures the Shuffleboard dashboard "Shooter" tab */
  private void configureShuffleboard() {
    // Create references to Shooter tab and its various layouts
    ShuffleboardTab shuffleShooterTab = Shuffleboard.getTab("Shooter");

    // Add piston object to tab
    shuffleShooterTab
        .add("Trigger Piston", shooterPCM)
        .withPosition(0, 0)
        .withSize(2, 1);

    // Configure speed slider
    shooterSpeed = shuffleShooterTab
        .add("Shooter Speed", SHOOTER_SPEED)
        .withWidget(kNumberSlider)
        .withProperties(
            Map.of(
                "Min", 0,
                "Max", 1))
        .withSize(2, 1)
        .withPosition(0, 1)
        .getEntry();
  }

  /* Toggle the trigger on/off */
  public void toggleTrigger(boolean offOn){
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

  /** Set the motor speed to the default in Constants */
  public void setMotorSpeed() {
    setMotorSpeed(SHOOTER_SPEED);
  }

  /** Set the motor speed between [0.0, 1.0] */
  public void setMotorSpeed(double speed) {
    shooterSpeed.setDouble(speed);
  }

  /** Set the motor speed to the default for auton mode */
  public void setMotorSpeedAuton() {
    setMotorSpeed(1.0);
  }

  @Override
  public void periodic() {
    double output = (motorsOn) ? shooterSpeed.getDouble(SHOOTER_SPEED) : 0;
    front.set(output);
    rear.set(output);
  }
}
