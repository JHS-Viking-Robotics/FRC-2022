// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import frc.robot.Constants;
import static frc.robot.Constants.Subsystem.Drivetrain.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import static com.revrobotics.SparkMaxRelativeEncoder.Type.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import static edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax leftFront;              // Left side front motor
  private final CANSparkMax leftBack;               // Left side rear motor
  private final CANSparkMax rightFront;             // Right side front motor
  private final CANSparkMax rightBack;              // Right side rear motor
  private final RelativeEncoder leftFrontEncoder;   // Left side front encoder
  private final RelativeEncoder leftBackEncoder;    // Left side rear encoder
  private final RelativeEncoder rightFrontEncoder;  // Right side front encoder
  private final RelativeEncoder rightBackEncoder;   // Right side rear encoder
  private final MecanumDrive driveMecanum;          // Mecanum drive interface
  private final ADXRS450_Gyro driveGyro;            // Gyroscope for determining robot heading
  private final MecanumDriveOdometry driveOdometryMec; // MecanumDrive Odometry object for keeping track of robot Pose

  private NetworkTableEntry leftDistance;   // NetworkTables odometer for left side sensors
  private NetworkTableEntry rightDistance;  // NetworkTables odometer for right side sensors
  private NetworkTableEntry leftVelocity;   // NetworkTables speedometer for left side sensors
  private NetworkTableEntry rightVelocity;  // NetworkTables speedometer for right side sensors

  private NetworkTableEntry driveP;         // kP for Talon closed-loop PID
  private NetworkTableEntry driveI;         // kI for Talon closed-loop PID
  private NetworkTableEntry driveD;         // kD for Talon closed-loop PID
  private NetworkTableEntry driveF;         // kF for Talon closed-loop PID
  private NetworkTableEntry driveSpeed;     // Percent of max output for motors

  private double loopCount;                 // Number of control loops ran

  /** Hopper Intake modes of operation */
  public enum Motors {
    /** Left Front drive motor */
    LEFT_FRONT,
    /** Left Back drive motor */
    LEFT_BACK,
    /** Right Front drive motor */
    RIGHT_FRONT,
    /** Right Back drive motor */
    RIGHT_BACK,
    /** Both Right side drive motors */
    RIGHT,
    /** Both Left side drive motors */
    LEFT,
    /** All drive motors together */
    ALL
  }


  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Connect to the motor controllers
    leftFront = new CANSparkMax(LEFT_FRONT_ID, MotorType.kBrushless);
    leftBack = new CANSparkMax(LEFT_BACK_ID, MotorType.kBrushless);
    rightFront = new CANSparkMax(RIGHT_FRONT_ID, MotorType.kBrushless);
    rightBack = new CANSparkMax(RIGHT_BACK_ID, MotorType.kBrushless);

    // Reset the controllers and set the inversion
    leftFront.restoreFactoryDefaults();
    leftBack.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();
    rightBack.restoreFactoryDefaults();
    leftFront.setInverted(LEFT_FRONT_INVERTED);
    leftBack.setInverted(LEFT_BACK_INVERTED);
    rightFront.setInverted(RIGHT_FRONT_INVERTED);
    rightBack.setInverted(RIGHT_BACK_INVERTED);

    // Connect to the encoders on the controllers, and set the conversion rate
    // for meters and meters/second
    leftFrontEncoder = leftFront.getEncoder(kHallSensor, 42);
    leftBackEncoder = leftBack.getEncoder(kHallSensor, 42);
    rightFrontEncoder = rightFront.getEncoder(kHallSensor, 42);
    rightBackEncoder = rightBack.getEncoder(kHallSensor, 42);
    leftFrontEncoder.setPositionConversionFactor(
        Constants.Chassis.WHEEL_CIRCUM / 10.71);
    leftBackEncoder.setPositionConversionFactor(
        Constants.Chassis.WHEEL_CIRCUM / 10.71);
    rightFrontEncoder.setPositionConversionFactor(
        Constants.Chassis.WHEEL_CIRCUM / 10.71);
    rightBackEncoder.setPositionConversionFactor(
        Constants.Chassis.WHEEL_CIRCUM / 10.71);
    leftFrontEncoder.setVelocityConversionFactor(
        Constants.Chassis.WHEEL_CIRCUM / (10.71 * 60));
    leftBackEncoder.setVelocityConversionFactor(
        Constants.Chassis.WHEEL_CIRCUM / (10.71 * 60));
    rightFrontEncoder.setVelocityConversionFactor(
        Constants.Chassis.WHEEL_CIRCUM / (10.71 * 60));
    rightBackEncoder.setVelocityConversionFactor(
        Constants.Chassis.WHEEL_CIRCUM / (10.71 * 60));

    // Connect to and reset the gyroscope
    driveGyro = new ADXRS450_Gyro();
    driveGyro.reset();

    // Set up the MecanumDrive and the Odometry. These are helper classes which
    // let us easily drive Mecanum-style and track our robot on the field
    driveMecanum = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
    driveOdometryMec = new MecanumDriveOdometry(MEC_KINEMATICS, getGyroRotation());

    // Configure the Shuffleboard tab for the Drivetain
    configureShuffleboard();
  }

  /** Configures the Shuffleboard dashboard "Drivetrain" tab */
  private void configureShuffleboard() {
    // Create references to Drivetrain tab and its various layouts
    ShuffleboardTab shuffleDrivetrainTab = Shuffleboard.getTab("Drivetrain");
    ShuffleboardLayout shufflePIDLayout = shuffleDrivetrainTab
        .getLayout("PID", BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "LEFT"))
        .withPosition(5, 0)
        .withSize(1, 4);
    ShuffleboardLayout shuffleDistanceLayout = shuffleDrivetrainTab
        .getLayout("Odometer (m)", BuiltInLayouts.kGrid)
        .withProperties(Map.of(
            "Label position",    "BOTTOM",
            "Number of columns", 2,
            "Number of rows",    1))
        .withPosition(0, 0)
        .withSize(2, 1);
    // Note that NetworkTables uses a flat namespace where the keys use '/' to
    // denote hierarchy, so we can't write m/s for units
    ShuffleboardLayout shuffleVelocityLayout = shuffleDrivetrainTab
        .getLayout("Speedometer (ms)", BuiltInLayouts.kGrid)
        .withProperties(Map.of(
            "Label position",    "BOTTOM",
            "Number of columns", 2,
            "Number of rows",    1))
        .withPosition(0, 1)
        .withSize(2, 1);

    // Add drivetrain objects to tab
    shuffleDrivetrainTab
        .add("Mecanum Drivetrain", driveMecanum)
        .withPosition(2, 0)
        .withSize(3, 4);
    shuffleDrivetrainTab
        .add("Gyro Angle (degrees)", driveGyro)
        .withPosition(0,2)
        .withSize(2,2);

    // Configure PID list widget, and set default values from Constants
    driveP = shufflePIDLayout
        .add("P", P)
        .withWidget(kTextView)
        .getEntry();
    driveI = shufflePIDLayout
        .add("I", I)
        .withWidget(kTextView)
        .getEntry();
    driveD = shufflePIDLayout
        .add("D", D)
        .withWidget(kTextView)
        .getEntry();
    driveF = shufflePIDLayout
        .add("F", F)
        .withWidget(kTextView)
        .getEntry();

    // Set up the encoder indicators
    leftDistance = shuffleDistanceLayout
        .add("Left", 0)
        .withWidget(kTextView)
        .getEntry();
    rightDistance = shuffleDistanceLayout
        .add("Right", 0)
        .withWidget(kTextView)
        .getEntry();
    leftVelocity = shuffleVelocityLayout
        .add("Left", 0)
        .withWidget(kTextView)
        .getEntry();
    rightVelocity = shuffleVelocityLayout
        .add("Right", 0)
        .withWidget(kTextView)
        .getEntry();

    // Add a control for the drive speed
    driveSpeed = shuffleDrivetrainTab
        .add("Drive Speed", 0.2)
        .withWidget(kNumberSlider)
        .withProperties(
          Map.of(
              "Min", 0,
              "Max", 1))
      .withSize(2, 1)
      .withPosition(6, 0)
      .getEntry();
  }

  /** Get the left encoder total distance travelled in meters */
  public double getDistance(Motors motor) {
    switch (motor) {
      case LEFT_FRONT:
        return leftFrontEncoder.getPosition();
      case LEFT_BACK:
        return leftBackEncoder.getPosition();
      case RIGHT_FRONT:
        return rightFrontEncoder.getPosition();
      case RIGHT_BACK:
        return rightBackEncoder.getPosition();
      case LEFT:
        return (getDistance(Motors.LEFT_FRONT) + getDistance(Motors.LEFT_BACK)) / 2.0;
      case RIGHT:
        return (getDistance(Motors.RIGHT_FRONT) + getDistance(Motors.RIGHT_BACK)) / 2.0;
      case ALL:
        return (getDistance(Motors.LEFT) + getDistance(Motors.RIGHT)) / 2.0;
      default:
        throw new IllegalArgumentException("Illegal arg to Drivetrain.getDiscance(Motors)");
    }
  }

  /** Get the left encoder velocity in meters per second */
  public double getVelocity(Motors motor) {
    switch (motor) {
      case LEFT_FRONT:
        return leftFrontEncoder.getVelocity();
      case LEFT_BACK:
        return leftBackEncoder.getVelocity();
      case RIGHT_FRONT:
        return rightFrontEncoder.getVelocity();
      case RIGHT_BACK:
        return rightBackEncoder.getVelocity();
      case LEFT:
        return (getVelocity(Motors.LEFT_FRONT) + getVelocity(Motors.LEFT_BACK)) / 2.0;
      case RIGHT:
        return (getVelocity(Motors.RIGHT_FRONT) + getVelocity(Motors.RIGHT_BACK)) / 2.0;
      case ALL:
        return (getVelocity(Motors.LEFT) + getVelocity(Motors.RIGHT)) / 2.0;
      default:
        throw new IllegalArgumentException("Illegal arg to Drivetrain.getDiscance(Motors)");
    }
  }

  /** Get the current MecanumDrive WheelSpeeds of the robot */
  public MecanumDriveWheelSpeeds getWheelSpeedsMec() {
    return new MecanumDriveWheelSpeeds(
        getVelocity(Motors.LEFT_FRONT),
        getVelocity(Motors.RIGHT_FRONT),
        getVelocity(Motors.LEFT_BACK),
        getVelocity(Motors.RIGHT_BACK));
  }

  /** Reset the encoders to zero */
  public void resetEncoders() {
    resetEncoders(0);
  }

  /** Reset the encoders to the position */
  public void resetEncoders(double position) {
    leftFront.getEncoder().setPosition(position);
    leftBack.getEncoder().setPosition(position);
    rightFront.getEncoder().setPosition(position);
    rightBack.getEncoder().setPosition(position);
  }

  /** Get the current rotational heading from the Gyroscope in degrees [-180, 180] */
  public double getGyroAngle() {
    return driveGyro.getRotation2d().getDegrees();
  }

  /** Get the current rotational velocity in degrees per second */
  public double getGyroVelocity() {
    return driveGyro.getRate();
  }

  /** Get the current rotational heading from the Gyroscope as a Rotation2d */
  public Rotation2d getGyroRotation() {
    return driveGyro.getRotation2d();
  }

  /** Reset the Gyroscope heading to 0. Note that this won't work correctly if
   * the robot is moving, and the subsystem will refuse to perform the reset
   * while moving
   */
  public void resetGyro() {
    if (getVelocity(Motors.LEFT) != 0 || getVelocity(Motors.RIGHT) != 0) {
      System.out.println("WARNING: Do not try to reset the gyroscope while the robot is moving");
      return;
    }
    driveGyro.reset();
  }

  /** Return the current estimated MecanumDrive Pose2d of the robot */
  public Pose2d getPoseMec() {
    return driveOdometryMec.getPoseMeters();
  }

  /** Reset the MecanumDrive odometry model to the specified Pose2d */
  public void resetOdometryMec(Pose2d pose) {
    resetEncoders();
    driveOdometryMec.resetPosition(pose, getGyroRotation());
  }

  /**
   * Reset the MecanumDrive odometry model and the encoders
   *
   * @see #resetOdometryMec(Pose2d)
   */
  public void resetOdometryMec() {
    resetOdometryMec(new Pose2d(0, 0, getGyroRotation()));
  }

  /** Drives the robot using ArcadeDrive style control */
  public void arcadeDrive(double throttle, double rotation) {
    driveMecanum.driveCartesian(throttle, 0, rotation);
  }

  /** Move in the given direction */
  public void moveTo(Translation2d position) {
    // Convert the x and y components into unit vectors, ignoring a component if
    // it is below 5 centimeters. Note Translation2d uses forward +x left +y,
    // and driveCartesian uses right +x forward +y
    double x = (Math.abs(position.getX()) > 0.05) ? (position.getX() / position.getNorm()) : 0.0;
    double y = (Math.abs(position.getY()) > 0.05) ? (position.getY() / position.getNorm()) : 0.0;
    System.out.println("Moving " + x + ", " + y);
    driveMecanum.driveCartesian(
        x * driveSpeed.getDouble(0.0),
        -y * driveSpeed.getDouble(0.0),
        0.0);
  }

  /** Mecanum drive */
  public void mecanumDrive(double throttle, double slide, double rotation) {
    driveMecanum.driveCartesian(
        driveSpeed.getDouble(0.0) * throttle,
        driveSpeed.getDouble(0.0) * slide,
        driveSpeed.getDouble(0.0) * rotation);
  }

  /** Mecanum drive with Field-Oriented Driving */
  public void mecanumDriveFOD(double throttle, double slide, double rotation) {
    driveMecanum.driveCartesian(
        driveSpeed.getDouble(0.0) * throttle,
        driveSpeed.getDouble(0.0) * slide,
        driveSpeed.getDouble(0.0) * rotation,
        getGyroAngle());
  }

  /** Tank drive using voltage output to the motors */
  public void tankDriveVoltage(double left, double right) {
    leftFront.setVoltage(left);
    leftBack.setVoltage(left);
    rightFront.setVoltage(right);
    rightBack.setVoltage(right);
  }

  /**
   * Update the PIDF configuration for both encoders from the NetworkTables values
   * configured on the Shuffleboard
   */
  public void syncNetworkTables() {
    if (++loopCount % 50 == 0) {
      // Push Drivetrain PIDF constants from NetworkTables to the controllers
      leftFront.getPIDController().setP(driveP.getDouble(P));
      leftFront.getPIDController().setI(driveI.getDouble(I));
      leftFront.getPIDController().setD(driveD.getDouble(D));
      leftFront.getPIDController().setFF(driveF.getDouble(F));
      leftBack.getPIDController().setP(driveP.getDouble(P));
      leftBack.getPIDController().setI(driveI.getDouble(I));
      leftBack.getPIDController().setD(driveD.getDouble(D));
      leftBack.getPIDController().setFF(driveF.getDouble(F));
      rightFront.getPIDController().setP(driveP.getDouble(P));
      rightFront.getPIDController().setI(driveI.getDouble(I));
      rightFront.getPIDController().setD(driveD.getDouble(D));
      rightFront.getPIDController().setFF(driveF.getDouble(F));
      rightBack.getPIDController().setP(driveP.getDouble(P));
      rightBack.getPIDController().setI(driveI.getDouble(I));
      rightBack.getPIDController().setD(driveD.getDouble(D));
      rightBack.getPIDController().setFF(driveF.getDouble(F));
    }

    // Push the current potition data from the sensors to the NetworkTables
    leftDistance.setDouble(getDistance(Motors.LEFT));
    rightDistance.setDouble(getDistance(Motors.RIGHT));
    leftVelocity.setDouble(getVelocity(Motors.LEFT));
    rightVelocity.setDouble(getVelocity(Motors.RIGHT));
  }

  @Override
  public void periodic() {
    driveOdometryMec.update(getGyroRotation(), getWheelSpeedsMec());
    syncNetworkTables();
  }
}
