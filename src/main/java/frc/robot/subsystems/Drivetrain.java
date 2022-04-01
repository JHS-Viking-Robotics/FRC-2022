// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Subsystem.Drivetrain.*;
import static frc.robot.Constants.Chassis.*;
import static com.revrobotics.CANSparkMax.IdleMode.*;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax; 
import com.revrobotics.RelativeEncoder; 
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax leftFront;  // Left side front SparkMAX controller
  private final CANSparkMax rightFront; // Left side rear SparkMAX controller
  private final CANSparkMax leftRear;   // Right side front SparkMAX controller
  private final CANSparkMax rightRear;  // Right side rearSparkMAX controller
  
  private final MecanumDrive driveMecanum; // MecanumDrive object, use this for driving
  /* MecanumDrive odometry object, use for estimating robot position on the field */
  private final MecanumDriveOdometry driveOdometry;
  private final ADXRS450_Gyro driveGyro;   // Gyroscope on the RoboRIO
  
  private final RelativeEncoder leftFrontEncoder;  // Left side front encoder 
  private final RelativeEncoder leftRearEncoder;   // Left side rear encoder 
  private final RelativeEncoder rightFrontEncoder; // Right side front encoder 
  private final RelativeEncoder rightRearEncoder;  // Right side rear encoder
  
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Initialize the motor controllers and connect to them
    leftFront = new CANSparkMax(LEFT_FRONT_ID, kBrushless);
    rightFront = new CANSparkMax(RIGHT_FRONT_ID, kBrushless);
    leftRear = new CANSparkMax(LEFT_BACK_ID, kBrushless);
    rightRear = new CANSparkMax(RIGHT_BACK_ID, kBrushless);

    // Reset the controllers and set their inversion
    leftFront.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();
    leftRear.restoreFactoryDefaults();
    rightRear.restoreFactoryDefaults();

    leftFront.setInverted(LEFT_FRONT_INVERTED);
    rightFront.setInverted(RIGHT_FRONT_INVERTED);
    leftRear.setInverted(LEFT_BACK_INVERTED);
    rightRear.setInverted(RIGHT_BACK_INVERTED);

    // Set up the onboard encoders
    leftFrontEncoder = leftFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42); 
    leftRearEncoder = leftRear.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42); 
    rightFrontEncoder = rightFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42); 
    rightRearEncoder = rightRear.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    leftFrontEncoder.setPositionConversionFactor(WHEEL_CIRCUM / 10.71); 
    leftRearEncoder.setPositionConversionFactor(WHEEL_CIRCUM / 10.71); 
    rightFrontEncoder.setPositionConversionFactor(WHEEL_CIRCUM / 10.71);
    rightRearEncoder.setPositionConversionFactor(WHEEL_CIRCUM / 10.71);
    leftFrontEncoder.setVelocityConversionFactor(WHEEL_CIRCUM / (10.71 * 60));
    leftRearEncoder.setVelocityConversionFactor(WHEEL_CIRCUM / (10.71 * 60)); 
    rightFrontEncoder.setVelocityConversionFactor(WHEEL_CIRCUM / (10.71 * 60)); 
    rightRearEncoder.setVelocityConversionFactor(WHEEL_CIRCUM / (10.71 * 60));

    // Configure MecanumDrive object and set up the gyroscope, also set up the odometry
    driveMecanum = new MecanumDrive(leftFront, leftRear, rightFront, rightRear);
    driveGyro = new ADXRS450_Gyro();
    driveGyro.reset();
    driveOdometry = new MecanumDriveOdometry(KINEMATICS, getGyroRotation());
  }

  /** Drives the robot using ArcadeDrive style control */
  public void arcadeDrive(double throttle, double rotation) {
    driveMecanum.driveCartesian(throttle, 0, rotation);
  }

  /**
   * Drive Mecanum style with/without FOD (Field Oriented Driving)
   * 
   * @param throttle speed along y axis (forward is positive) [-1.0, 1.0]
   * @param slide speed along x-axis (right is positive) [-1.0, 1.0]
   * @param rotation rotational speed (clockwise is positive) [-1.0, 1.0]
   * @param useFOD use Field Oriented Driving
   */ 
  public void drive(double throttle, double slide, double rotation, boolean useFOD) { 
    if (useFOD) {
      driveMecanum.driveCartesian(throttle, slide, rotation, getGyroAngle());
    } else {
      driveMecanum.driveCartesian(throttle, slide, rotation);
    }
  }

  /**
   * Sets the drive motors to brake mode
   * @see #setCoast
   */
  public void setBrake() {
    leftFront.setIdleMode(kBrake);
  }

  /**
   * Sets the drive motors to brake mode
   * @see #setBrake
   */
  public void setCoast() {
    leftFront.setIdleMode(kCoast);
  }

  /** Set the max speed of the drivetrain between [0,1] */
  public void setMaxSpeed(double maxSpeed) {
    driveMecanum.setMaxOutput(maxSpeed);
  }

  /** Set the max speed of the drivetrain between [0,1] */
  public void setTurboSpeed() {
    driveMecanum.setMaxOutput(1.0);
  }

  /**
   * Reset the max speed of the drivetrain to the default in Constants.
   * 
   * @see #setMaxSpeed(double)
   */
  public void setMaxSpeed() {
    driveMecanum.setMaxOutput(MAX_SPEED);
  }

  /** Reset the encoders to 0 */
  public void resetEncoder(){
    leftFront.getEncoder().setPosition(0);
    rightFront.getEncoder().setPosition(0);
    leftRear.getEncoder().setPosition(0);
    rightRear.getEncoder().setPosition(0); 
  }

  /** Get the distance driven by the left side */
  public double getDistanceLeft() {
    return (leftFrontEncoder.getPosition() + leftRearEncoder.getPosition())/2.0;
  }

  /** Get the distance driven by the right sice */
  public double getDistanceRight() {
    return (rightFrontEncoder.getPosition() + rightRearEncoder.getPosition())/2.0;
  }

  /** Get the velocity of the left side */
  public double getVelocityLeft(){
    return (leftFrontEncoder.getVelocity() + leftRearEncoder.getVelocity())/2.0;
  }

  /** Get the velocity of the right side */
  public double getVelocityRight(){
    return (rightFrontEncoder.getVelocity() + rightRearEncoder.getVelocity())/2.0;
  }

  /**
   * Returns the current wheel speeds of the drivetrain.
   */
  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        leftFrontEncoder.getVelocity(),
        rightFrontEncoder.getVelocity(),
        leftRearEncoder.getVelocity(),
        rightRearEncoder.getVelocity());
  }

  /** Get the current robot pose as a Pose2d object in meters */
  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }
  /** Get the current gyroscope angle in degrees from forward [-180, 180] */
  public double getGyroAngle() {
    return driveGyro.getRotation2d().getDegrees();
  }

  /** Get the current gyroscope velocity in degrees/second,
   * where positive is counter-clockwise */
  public double getGyroVelocity() {
    return -driveGyro.getRate();
  }

  /** Get the current gyroscope rotation as a Rotation2d object */
  public Rotation2d getGyroRotation() {
    return driveGyro.getRotation2d();
  }

  /** Reset the gyroscope to 0 */
  public void resetGyro() {
    if (getVelocityLeft() != 0 || getVelocityRight() != 0) {
     System.out.println("WARNING: Do not try to reset the gyroscope while the robot is moving");
     return;
   }
    driveGyro.reset();
  }

  @Override
  public void periodic() {
    // Update odometry
    driveOdometry.update(getGyroRotation(), getWheelSpeeds());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}