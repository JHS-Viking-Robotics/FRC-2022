// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.package frc.robot.subsystems;

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  /** Creates a new Lift. */
  private final CANSparkMax left;
  public  Lift() {
    left = new CANSparkMax(Constants.Subsystem.Lift.LEFT_ID, MotorType.kBrushless); 

  left.restoreFactoryDefaults();
left.setInverted(Constants.Subsystem.Lift.LEFT_INVERTED); 
  }

  public void goUp(){
    left.set(0.5);
  }
  public void goDown(){
    left.set(-0.5);
  }
  public void stop(){
    left.set(0);
  }
}