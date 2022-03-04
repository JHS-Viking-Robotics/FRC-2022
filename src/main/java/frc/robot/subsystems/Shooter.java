// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Subsystem.Shooter.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final CANSparkMax topLeft;      // Top left shooter motor
  private final CANSparkMax topRight;     // Top right shooter motor
  private final CANSparkMax bottomLeft;   // Bottom left shooter motor
  private final CANSparkMax bottomRight;  // Bottom right shooter motor
  private final DoubleSolenoid trigger;   // Piston to push balls into firing position

  /** Creates a new Shooter. */
  public Shooter() {
    // Connect to the motor controllers and piston
    topLeft = new CANSparkMax(TOP_LEFT_ID, MotorType.kBrushless);
    topRight = new CANSparkMax(TOP_RIGHT_ID, MotorType.kBrushless);
    bottomLeft = new CANSparkMax(BOTTOM_LEFT_ID, MotorType.kBrushless);
    bottomRight = new CANSparkMax(BOTTOM_RIGHT_ID, MotorType.kBrushless);
    trigger = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    // Configure the motors. Positive direction should be out
    topLeft.restoreFactoryDefaults();
    topRight.restoreFactoryDefaults();
    bottomLeft.restoreFactoryDefaults();
    bottomRight.restoreFactoryDefaults();
    topLeft.setInverted(TOP_LEFT_INVERTED);
    topRight.setInverted(TOP_RIGHT_INVERTED);
    bottomLeft.setInverted(BOTTOM_LEFT_INVERTED);
    bottomRight.setInverted(BOTTOM_RIGHT_INVERTED);

    // Piston should start retracted
    trigger.set(kReverse);
  }

  /*
   * Toggle the firing motors on or off. Note that this may take a second to
   * take effect as the motors spin up
   */
  public void toggleMotors(boolean active) {
    double output = active ? 100 : 0;
    topLeft.set(output);
    topRight.set(output);
    bottomLeft.set(output);
    bottomRight.set(output);
  }

  /*
   * Toggle the firing trigger in or out
   */
  public void toggleTrigger(boolean active) {
    if (active) { trigger.set(kForward); }
    else { trigger.set(kReverse); }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
