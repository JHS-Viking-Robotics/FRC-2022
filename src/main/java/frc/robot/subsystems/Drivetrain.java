// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import frc.robot.Constants;
import frc.robot.Constants.Subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax leftFront;      // Left side front motor
  private final CANSparkMax leftBack;       // Left side rear motor
  private final CANSparkMax rightFront;     // Right side front motor
  private final CANSparkMax rightBack;      // Right side rear motor
  private final RelativeEncoder leftFrontEncoder;  // Left side front encoder
  private final RelativeEncoder leftBackEncoder;   // Left side rear encoder
  private final RelativeEncoder rightFrontEncoder; // Right side front encoder
  private final RelativeEncoder rightBackEncoder;  // Right side rear encoder
  private final MecanumDrive driveMecanum;  // Mecanum drive interface
  private final ADXRS450_Gyro driveGyro;    // Gyroscope for determining robot heading
  private final MecanumDriveOdometry driveOdometry; // Odometry object for keeping track of robot Pose

  private NetworkTableEntry leftDistance;   // NetworkTables odometer for left side sensors
  private NetworkTableEntry rightDistance;  // NetworkTables odometer for right side sensors
  private NetworkTableEntry leftVelocity;   // NetworkTables speedometer for left side sensors
  private NetworkTableEntry rightVelocity;  // NetworkTables speedometer for right side sensors

  private NetworkTableEntry driveP;         // kP for Talon closed-loop PID
  private NetworkTableEntry driveI;         // kI for Talon closed-loop PID
  private NetworkTableEntry driveD;         // kD for Talon closed-loop PID
  private NetworkTableEntry driveF;         // kF for Talon closed-loop PID
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

  private final double driveSpeedScalar = 0.6; // Percent of max output for motors

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFront = new CANSparkMax(Subsystem.Drivetrain.LEFT_FRONT_ID, MotorType.kBrushless);
    rightFront = new CANSparkMax(Subsystem.Drivetrain.LEFT_BACK_ID, MotorType.kBrushless);
    leftBack = new CANSparkMax(Subsystem.Drivetrain.RIGHT_FRONT_ID, MotorType.kBrushless);
    rightBack = new CANSparkMax(Subsystem.Drivetrain.RIGHT_BACK_ID, MotorType.kBrushless);

    leftFront.restoreFactoryDefaults();
    leftBack.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();
    rightBack.restoreFactoryDefaults();

    leftFront.setInverted(Subsystem.Drivetrain.LEFT_FRONT_INVERTED);
    leftBack.setInverted(Subsystem.Drivetrain.LEFT_BACK_INVERTED);
    rightFront.setInverted(Subsystem.Drivetrain.RIGHT_FRONT_INVERTED);
    rightBack.setInverted(Subsystem.Drivetrain.RIGHT_BACK_INVERTED);

    leftFrontEncoder = leftFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    leftBackEncoder = leftBack.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    rightFrontEncoder = rightFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    rightBackEncoder = rightBack.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    leftFrontEncoder.setPositionConversionFactor(
        (10.71 * 42.0) / Constants.Chassis.WHEEL_DIAMETER);
    leftBackEncoder.setPositionConversionFactor(
        (10.71 * 42.0) / Constants.Chassis.WHEEL_DIAMETER);
    rightFrontEncoder.setPositionConversionFactor(
        (10.71 * 42.0) / Constants.Chassis.WHEEL_DIAMETER);
    rightBackEncoder.setPositionConversionFactor(
        (10.71 * 42.0) / Constants.Chassis.WHEEL_DIAMETER);
    leftFrontEncoder.setVelocityConversionFactor(
        (10.71 * 42.0) / Constants.Chassis.WHEEL_DIAMETER);
    leftBackEncoder.setVelocityConversionFactor(
        (10.71 * 42.0) / Constants.Chassis.WHEEL_DIAMETER);
    rightFrontEncoder.setVelocityConversionFactor(
        (10.71 * 42.0) / Constants.Chassis.WHEEL_DIAMETER);
    rightBackEncoder.setVelocityConversionFactor(
        (10.71 * 42.0) / Constants.Chassis.WHEEL_DIAMETER);

    driveMecanum = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
    driveGyro = new ADXRS450_Gyro();
    driveGyro.reset();
    driveOdometry = new MecanumDriveOdometry(Subsystem.Drivetrain.KINEMATICS, getGyroRotation());

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
    // denote hierarchy, so we manually set the title in Shuffleboard instead
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
        .add("P", Subsystem.Drivetrain.P)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    driveI = shufflePIDLayout
        .add("I", Subsystem.Drivetrain.I)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    driveD = shufflePIDLayout
        .add("D", Subsystem.Drivetrain.D)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    driveF = shufflePIDLayout
        .add("F", Subsystem.Drivetrain.F)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    // Set up the encoder indicators
    leftDistance = shuffleDistanceLayout
        .add("Left", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    rightDistance = shuffleDistanceLayout
        .add("Right", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    leftVelocity = shuffleVelocityLayout
        .add("Left", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    rightVelocity = shuffleVelocityLayout
        .add("Right", 0)
        .withWidget(BuiltInWidgets.kTextView)
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

  /** Get the current WheelSpeeds of the robot */
  public MecanumDriveWheelSpeeds getWheelSpeeds() {
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

  /** Return the current estimated Pose2d of the robot */
  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }

  /** Reset the odometry model to the specified Pose2d */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    driveOdometry.resetPosition(pose, getGyroRotation());
  }

  /**
   * Reset the odometry model and the encoders
   *
   * @see #resetOdometry(Pose2d)
   */
  public void resetOdometry() {
    resetOdometry(getPose());
  }

  /** Drives the robot using ArcadeDrive style control */
  public void arcadeDrive(double throttle, double rotation) {
    driveMecanum.driveCartesian(driveSpeedScalar * throttle, 0, driveSpeedScalar * rotation);
  }

  /** Mecanum drive */
  public void mecanumDrive(double throttle, double slide, double rotation) {
    driveMecanum.driveCartesian(driveSpeedScalar * throttle,
        driveSpeedScalar * slide, driveSpeedScalar * rotation);
  }

  /** Mecanum drive with Field-Oriented Driving */
  public void mecanumDriveFOD(double throttle, double slide, double rotation) {
    driveMecanum.driveCartesian(driveSpeedScalar * throttle,
        driveSpeedScalar * slide, driveSpeedScalar * rotation, getGyroAngle());
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
    // Push Drivetrain PIDF constants from NetworkTables to the controllers
    leftFront.getPIDController().setP(driveP.getDouble(Subsystem.Drivetrain.P));
    leftFront.getPIDController().setI(driveI.getDouble(Subsystem.Drivetrain.I));
    leftFront.getPIDController().setD(driveD.getDouble(Subsystem.Drivetrain.D));
    leftFront.getPIDController().setFF(driveF.getDouble(Subsystem.Drivetrain.F));
    leftBack.getPIDController().setP(driveP.getDouble(Subsystem.Drivetrain.P));
    leftBack.getPIDController().setI(driveI.getDouble(Subsystem.Drivetrain.I));
    leftBack.getPIDController().setD(driveD.getDouble(Subsystem.Drivetrain.D));
    leftBack.getPIDController().setFF(driveF.getDouble(Subsystem.Drivetrain.F));
    rightFront.getPIDController().setP(driveP.getDouble(Subsystem.Drivetrain.P));
    rightFront.getPIDController().setI(driveI.getDouble(Subsystem.Drivetrain.I));
    rightFront.getPIDController().setD(driveD.getDouble(Subsystem.Drivetrain.D));
    rightFront.getPIDController().setFF(driveF.getDouble(Subsystem.Drivetrain.F));
    rightBack.getPIDController().setP(driveP.getDouble(Subsystem.Drivetrain.P));
    rightBack.getPIDController().setI(driveI.getDouble(Subsystem.Drivetrain.I));
    rightBack.getPIDController().setD(driveD.getDouble(Subsystem.Drivetrain.D));
    rightBack.getPIDController().setFF(driveF.getDouble(Subsystem.Drivetrain.F));

    // Push the current potition data from the sensors to the NetworkTables
    leftDistance.setDouble(getDistance(Motors.LEFT));
    rightDistance.setDouble(getDistance(Motors.RIGHT));
    leftVelocity.setDouble(getVelocity(Motors.LEFT));
    rightVelocity.setDouble(getVelocity(Motors.RIGHT));
  }

  @Override
  public void periodic() {
    syncNetworkTables();
    driveOdometry.update(getGyroRotation(), getWheelSpeeds());
  }
}
