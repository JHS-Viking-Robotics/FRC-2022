// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.package frc.robot.subsystems;

package frc.robot.subsystems;

import static frc.robot.Constants.Subsystem.Lift.*;

import com.revrobotics.CANSparkMax; 
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {

  private final CANSparkMax lift; // 

  /** Creates a new Lift. */
  public  Lift() {
    lift = new CANSparkMax(LIFT_ID, kBrushless);
    lift.restoreFactoryDefaults();
    lift.setInverted(LIFT_INVERTED); 
  }

  public void goUp(){
    lift.set(0.5);
  }
  public void goDown(){
    lift.set(-0.5);
  }
  public void stop(){
    lift.set(0);
  }
}