// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax leftMain;
  private final CANSparkMax leftFollow;
  private final CANSparkMax rightMain;
  private final CANSparkMax rightFollow;
  private final DifferentialDrive diffDrive;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftMain = new CANSparkMax(1, MotorType.kBrushless);
    rightMain = new CANSparkMax(2, MotorType.kBrushless);
    leftFollow = new CANSparkMax(3, MotorType.kBrushless);
    rightFollow = new CANSparkMax(4, MotorType.kBrushless);

    leftFollow.follow(leftMain);
    rightFollow.follow(rightMain);

    diffDrive = new DifferentialDrive(leftMain, rightMain);

  }

  /** Drives the robot using ArcadeDrive style control */
  public void arcadeDrive(double throttle, double rotation) {
    diffDrive.arcadeDrive(throttle, rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
