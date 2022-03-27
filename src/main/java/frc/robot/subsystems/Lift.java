// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.package frc.robot.subsystems;

package frc.robot.subsystems;

import static frc.robot.Constants.Subsystem.Lift.*;
import static com.revrobotics.SparkMaxRelativeEncoder.Type.*;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {

  private final CANSparkMax lift;         // Motor controller for the Lift
  private final RelativeEncoder encoder;  // Encoder for the Lift motor

  /** Creates a new Lift. */
  public  Lift() {
    lift = new CANSparkMax(LIFT_ID, kBrushless);
    lift.restoreFactoryDefaults();
    lift.setInverted(LIFT_INVERTED); 
    encoder = lift.getEncoder(kHallSensor, 42);
    encoder.setPositionConversionFactor((2.0 * Math.PI * 0.015) / 16.0);
  }

  public void goUp(){
    set(LIFT_SPEED);
  }

  public void goDown(){
    set(-LIFT_SPEED);
  }

  public void stop(){
    set(0.0);
  }

  /** Set the output of the Lift */
  public void set(double speed) {
    lift.set(speed);
  }

  /** Get the distance in meters the Lift has travelled. Note that the Lift
   * normally only goes up and then "wraps around" so all the way up and back
   * down will be twice the height
   */
  public double getHeight() {
    return encoder.getPosition();
  }
  /** Reset the encoder to 0 */
  public void resetEncoder() {
    encoder.setPosition(0.0);
  }
}