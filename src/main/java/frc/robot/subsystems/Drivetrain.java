// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax; 
import com.revrobotics.RelativeEncoder; 
import com.revrobotics.SparkMaxRelativeEncoder; import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.subsystems.Drivetrain;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  /** ask Dan about compatiblity to mukanum drive*/
  private final CANSparkMax leftFront;
  private final CANSparkMax rightFront;
  private final CANSparkMax leftRear;
  private final CANSparkMax rightRear;
  private final MecanumDrive driveMecanum;
  private final ADXRS450_Gyro driveGyro;

  private final RelativeEncoder leftFrontEncoder; // Left side front encoder 
  private final RelativeEncoder leftBackEncoder; // Left side rear encoder 
  private final RelativeEncoder rightFrontEncoder; // Right side front encoder 
  private final RelativeEncoder rightBackEncoder; // Right side rear encoder



  public Drivetrain() {
    leftFront = new CANSparkMax(1, MotorType.kBrushless);
    rightFront = new CANSparkMax(1, MotorType.kBrushless);
    leftRear = new CANSparkMax(1, MotorType.kBrushless);
    rightRear = new CANSparkMax(1, MotorType.kBrushless);
    
    leftFront.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();
    leftRear.restoreFactoryDefaults();
    rightRear.restoreFactoryDefaults();

    leftFrontEncoder = leftFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42); leftBackEncoder = leftBack.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42); rightFrontEncoder = rightFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42); rightBackEncoder = rightBack.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    
    driveMecanum = new MecanumDrive(leftFront, leftRear, rightFront, rightRear);
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

  /** Mecanum drive with Field Oriented Driving */ 
  public void mecanumDriveFOD(double throttle, double slide, double rotation) { 
    driveMecanum.driveCartesian(throttle, slide, rotation, driveGyro.getRotation2d().getDegrees());
  }
public void resetEncoder(){
    leftFront.getEncoder().setPosition(0);
    rightFront.getEncoder().setPosition(0);
    leftRear.getEncoder().setPosition(0);
    rightRear.getEncoder().setPosition(0); 
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
