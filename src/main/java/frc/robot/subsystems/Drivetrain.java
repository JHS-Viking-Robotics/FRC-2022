// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax; 
import com.revrobotics.RelativeEncoder; 
import com.revrobotics.SparkMaxRelativeEncoder; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants;
import com.revrobotics.SparkMaxRelativeEncoder;

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
  private final RelativeEncoder leftRearEncoder; // Left side rear encoder 
  private final RelativeEncoder rightFrontEncoder; // Right side front encoder 
  private final RelativeEncoder rightRearEncoder; // Right side rear encoder



  public Drivetrain() {
    leftFront = new CANSparkMax(Constants.Subsystem.Drivetrain.LEFT_FRONT_ID, MotorType.kBrushless);
    rightFront = new CANSparkMax(Constants.Subsystem.Drivetrain.RIGHT_FRONT_ID, MotorType.kBrushless);
    leftRear = new CANSparkMax(Constants.Subsystem.Drivetrain.LEFT_BACK_ID, MotorType.kBrushless);
    rightRear = new CANSparkMax(Constants.Subsystem.Drivetrain.RIGHT_BACK_ID, MotorType.kBrushless);
    
    leftFront.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();
    leftRear.restoreFactoryDefaults();
    rightRear.restoreFactoryDefaults();
    
    leftFront.setInverted(Constants.Subsystem.Drivetrain.LEFT_FRONT_INVERTED);
    rightFront.setInverted(Constants.Subsystem.Drivetrain.RIGHT_FRONT_INVERTED);
    leftRear.setInverted(Constants.Subsystem.Drivetrain.LEFT_BACK_INVERTED);
    rightRear.setInverted(Constants.Subsystem.Drivetrain.RIGHT_BACK_INVERTED);
    

    leftFrontEncoder = leftFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42); 
    leftRearEncoder = leftRear.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42); 
    rightFrontEncoder = rightFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42); 
    rightRearEncoder = rightRear.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    leftFrontEncoder.setPositionConversionFactor( (10.71 * 42.0) / Constants.Chassis.WHEEL_CIRCUM); 
    leftRearEncoder.setPositionConversionFactor( (10.71 * 42.0) / Constants.Chassis.WHEEL_CIRCUM); 
    rightFrontEncoder.setPositionConversionFactor( (10.71 * 42.0) / Constants.Chassis.WHEEL_CIRCUM);
   rightRearEncoder.setPositionConversionFactor( (10.71 * 42.0) / Constants.Chassis.WHEEL_CIRCUM);
    leftFrontEncoder.setVelocityConversionFactor( (10.71 * 42.0) / Constants.Chassis.WHEEL_CIRCUM);
   leftRearEncoder.setVelocityConversionFactor( (10.71 * 42.0) / Constants.Chassis.WHEEL_CIRCUM); 
   rightFrontEncoder.setVelocityConversionFactor( (10.71 * 42.0) / Constants.Chassis.WHEEL_CIRCUM); 
   rightRearEncoder.setVelocityConversionFactor( (10.71 * 42.0) / Constants.Chassis.WHEEL_CIRCUM);



    
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
  public double getDistanceLeft() {
    return (leftFrontEncoder.getPosition() + leftRearEncoder.getPosition())/2.0;
  }
  public double getDistanceRight() {
    return (rightFrontEncoder.getPosition() + rightRearEncoder.getPosition())/2.0;
  }
public double getVelocityLeft(){
  return (leftFrontEncoder.getVelocity() + leftRearEncoder.getVelocity())/2.0;
}
public double getVelocityRight(){
  return (rightFrontEncoder.getVelocity() + rightRearEncoder.getVelocity())/2.0;
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
