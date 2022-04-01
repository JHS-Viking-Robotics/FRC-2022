// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.package frc.robot.subsystems;

package frc.robot.subsystems;

import static frc.robot.Constants.Subsystem.Lift.*;
import static com.revrobotics.CANSparkMax.IdleMode.*;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {

  private final CANSparkMax lift; // 

  /** Creates a new Lift. */
  public  Lift() {
    lift = new CANSparkMax(LIFT_ID, kBrushless);
    lift.restoreFactoryDefaults();
    lift.setInverted(LIFT_INVERTED);
    // Lift mechanism is spring loaded, and will try to lift itself a few inches
    // throughout the match. Using brake mode to counter this
    lift.setIdleMode(kBrake);
  }

  /** Raise the Lift */
  public void goUp() {
    lift.set(0.5);
  }

  /** Lower the Lift */
  public void goDown() {
    lift.set(-0.5);
  }

  /** Stop the Lift. Note that this will not work after the match when the power
   * is turned off. */
  public void stop() {
    lift.set(0);
  }
}