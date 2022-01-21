// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final CANSparkMax leftMain;
  private final CANSparkMax rightMain;
  private final CANSparkMax leftFollow;
  private final CANSparkMax rightFollow;


  public ExampleSubsystem() {
    leftMain = new CANSparkMax(1, MotorType.kBrushless);
    rightMain = new CANSparkMax(1, MotorType.kBrushless);
    leftFollow = new CANSparkMax(1, MotorType.kBrushless);
    rightFollow = new CANSparkMax(1, MotorType.kBrushless);

    leftMain.restoreFactoryDefults();
    rightMain.restoreFactoryDefults();
    leftFollow.restoreFactoryDefults();
    rightFollow.restoreFactoryDefults();

    leftFollow.follow(leftMain);
    rightFollow.follow(rightMain);
    


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
