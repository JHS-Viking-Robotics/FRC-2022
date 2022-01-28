// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax leftFront;
  private final CANSparkMax leftBack;
  private final CANSparkMax rightFront;
  private final CANSparkMax rightBack;
  private final MecanumDrive driveMecanum;
  private final ADXRS450_Gyro driveGyro;    // Gyroscope for determining robot heading


  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFront = new CANSparkMax(1, MotorType.kBrushless);
    rightFront = new CANSparkMax(2, MotorType.kBrushless);
    leftBack = new CANSparkMax(3, MotorType.kBrushless);
    rightBack = new CANSparkMax(4, MotorType.kBrushless);

    driveMecanum = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
    driveGyro = new ADXRS450_Gyro();
    driveGyro.reset();
  }

  /** Drives the robot using ArcadeDrive style control */
  public void arcadeDrive(double throttle, double rotation) {
    driveMecanum.driveCartesian(throttle, 0, rotation);
  }

  /** Mecanum drive */
  public void mecanumDrive(double throttle, double slide, double rotation) {
    driveMecanum.driveCartesian(throttle, slide, rotation);
  }

  /** Mecanum drive with Field-Oriented Driving */
  public void mecanumDriveFOD(double throttle, double slide, double rotation) {
    driveMecanum.driveCartesian(throttle, slide, rotation, getGyroAngle());
  }

  /** Get the angle of the gyroscope */
  public double getGyroAngle() {
    driveGryo.getRotation2d().getDegrees();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
