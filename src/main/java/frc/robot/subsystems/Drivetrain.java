// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.Chassis;
import frc.robot.Constants.Subsystem;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonSRX leftMain;     // Talon controller for left side primary motor
  private final WPI_VictorSPX leftFollow;  // Victor controller for left side follower motor
  private final WPI_TalonSRX rightMain;    // Talon controller for right side primary motor
  private final WPI_VictorSPX rightFollow; // Victor controller for right side follower motor
  private final Gyro driveGyro;            // Gyroscope for determining robot heading

  private final DifferentialDrive driveDifferential;
  private final DifferentialDriveKinematics driveKinematics;
  private final DifferentialDriveOdometry driveOdometry;
  private NetworkTableEntry leftDistance;  // NetworkTables odometer for left side sensors
  private NetworkTableEntry rightDistance; // NetworkTables odometer for right side sensors

  private NetworkTableEntry driveP; // kP for Talon closed-loop PID
  private NetworkTableEntry driveI; // kI for Talon closed-loop PID
  private NetworkTableEntry driveD; // kD for Talon closed-loop PID
  private NetworkTableEntry driveF; // kF for Talon closed-loop PID

  /**
   * Creates a new Drivetrain subsystem with 2 Talon and 2 Victor motor
   * controllers.
   */
  public Drivetrain() {
    // Initialize new Talon controllers and configure them
    leftMain = new WPI_TalonSRX(Subsystem.Drivetrain.LEFT_MAIN_ID);
    rightMain = new WPI_TalonSRX(Subsystem.Drivetrain.RIGHT_MAIN_ID);
    leftFollow = new WPI_VictorSPX(Subsystem.Drivetrain.LEFT_FOLLOW_ID);
    rightFollow = new WPI_VictorSPX(Subsystem.Drivetrain.RIGHT_FOLLOW_ID);
    driveGyro = new ADXRS450_Gyro();
    driveGyro.reset();
    configureTalons();
    
    // Configure differential drive, kinematics, and odometry
    driveDifferential = new DifferentialDrive(leftMain, rightMain);
    driveDifferential.setRightSideInverted(false);
    driveKinematics = new DifferentialDriveKinematics(Chassis.TRACK_WIDTH);
    driveOdometry = new DifferentialDriveOdometry(getGyroRotation());
    

    // Configure Shuffleboard dashboard tab and NetworkTable entries
    configureShuffleboard();
  }

  /** Configures the Talon motor controllers and safety settings */
  private void configureTalons() {
    // Set Talon inversion, encoder phase and type, and set followers
    leftMain.setInverted(Subsystem.Drivetrain.LEFT_CONTROLLER_INVERTED);
    leftMain.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    leftMain.setSensorPhase(Subsystem.Drivetrain.LEFT_SENSOR_INVERTED);

    rightMain.setInverted(Subsystem.Drivetrain.RIGHT_CONTROLLER_INVERTED);
    rightMain.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightMain.setSensorPhase(Subsystem.Drivetrain.RIGHT_SENSOR_INVERTED);

    leftFollow.setInverted(Subsystem.Drivetrain.LEFT_CONTROLLER_INVERTED);
    leftFollow.follow(leftMain);

    rightFollow.setInverted(Subsystem.Drivetrain.RIGHT_CONTROLLER_INVERTED);
    rightFollow.follow(rightMain);

    // Set Talon safety parameters. Note that Victor controllers do not have
    // these options, but by following the main controllers output they should
    // also stay in reasonable current output ranges
    leftMain.configFactoryDefault();
    leftMain.configPeakCurrentLimit(0);
    leftMain.configContinuousCurrentLimit(35);
    leftMain.configPeakCurrentDuration(100);
    leftMain.enableCurrentLimit(true);
    leftMain.setSafetyEnabled(true);

    rightMain.configFactoryDefault();
    rightMain.configPeakCurrentLimit(0);
    rightMain.configContinuousCurrentLimit(35);
    rightMain.configPeakCurrentDuration(100);
    rightMain.enableCurrentLimit(true);
    rightMain.setSafetyEnabled(true);
  }

  /** Configures the Shuffleboard dashboard "Drivetrain" tab */
  private void configureShuffleboard() {
    // Create references to Drivetrain tab and PID layout
    ShuffleboardTab shuffleDrivetrainTab = Shuffleboard.getTab("Drivetrain");
    ShuffleboardLayout shufflePIDLayout = shuffleDrivetrainTab
        .getLayout("PID", BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "LEFT"))
        .withPosition(3, 0)
        .withSize(1, 2);
    ShuffleboardLayout shuffleDistanceLayout = shuffleDrivetrainTab
        .getLayout("Odometer (m)", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Label position", "BOTTOM"))
        .withProperties(Map.of("Number of columns", 2))
        .withProperties(Map.of("Number of rows", 1))
        .withPosition(0, 3)
        .withSize(2, 1);

    // Add drivetrain objects to tab
    shuffleDrivetrainTab
        .add("Differential Drivetrain", driveDifferential)
        .withPosition(0, 0)
        .withSize(3, 3);

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
  }

  /** Get the left encoder total distance travelled in meters */
  public double getLeftDistance() {
    // Get the quadrature encoder position in ticks (4096 ticks/rotation)
    // Convert from raw ticks and return distance in meters
    return leftMain.getSelectedSensorPosition() * (Chassis.WHEEL_CIRCUM / 4096);
  }

  /** Get the left encoder velocity in meters per second */
  public double getLeftVelocity() {
   // Get the quadrature encoder position in ticks per 100ms
   // (4096 ticks/rotation). Then convert from raw ticks and return distance
   // in meters
    return leftMain.getSelectedSensorVelocity() * (1.0/10.0) * (Chassis.WHEEL_CIRCUM / 4096);
  }

  /** Get the right encoder total distance travelled in meters */
  public double getRightDistance() {
    // Get the quadrature encoder position in ticks (4096 ticks/rotation)
    // Convert from raw ticks and return distance in meters
    return rightMain.getSelectedSensorPosition() * (Chassis.WHEEL_CIRCUM / 4096);
  }

  /** Get the left encoder velocity in meters per second */
  public double getRightVelocity() {
    // Get the quadrature encoder position in ticks per 100ms
    // (4096 ticks/rotation). Then convert from raw ticks and return distance
    // in meters
     return rightMain.getSelectedSensorVelocity() * (1.0/10.0) * (Chassis.WHEEL_CIRCUM / 4096);
  }

  /** Get the average total distance travelled in meters from both encoders */
  public double getTotalDistance() {
    // Average the left and right encoder distances
    return (getRightDistance() + getLeftDistance()) / 2.0;
  }

  /** Get the average total encoder velocity in meters per second */
  public double getTotalVelocity() {
    // Average left and right velocities
    return (getLeftVelocity() + getRightDistance()) / 2.0;
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

  /** Return the current estimated Pose2d of the robot */
  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }

  /** Reset the encoders to 0 */
  public void resetEncoders() {
    leftMain.setSelectedSensorPosition(0);
    rightMain.setSelectedSensorPosition(0);
  }

  /** Reset the Gyroscope heading to 0. Note that this won't work correctly if
   * the robot is moving, and the subsystem will refuse to perform the reset
   * while moving
   */
  public void resetGyro() {
    if (getLeftVelocity() != 0 || getRightVelocity() != 0) {
      System.out.println("WARNING: Do not try to reset the gyroscope while the robot is moving");
      return;
    }
    driveGyro.reset();
  }

  /**
   * Reset the odometry model and the encoders
   *
   * @see #resetOdometry(Pose2d)
   */
  public void resetOdometry() {
    resetOdometry(getPose());
  }

  /** Reset the odometry model to the specified Pose2d */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    driveOdometry.resetPosition(pose, getGyroRotation());
  }

  /**
   * Update the PIDF configuration for both encoders from the NetworkTables values
   * configured on the Shuffleboard
   */
  public void syncNetworkTables() {
    // Push Drivetrain PIDF constants from NetworkTables to the controllers
    leftMain.config_kP(0, driveP.getDouble(Subsystem.Drivetrain.P));
    leftMain.config_kI(0, driveI.getDouble(Subsystem.Drivetrain.I));
    leftMain.config_kD(0, driveD.getDouble(Subsystem.Drivetrain.D));
    leftMain.config_kF(0, driveF.getDouble(Subsystem.Drivetrain.F));
    rightMain.config_kP(0, driveP.getDouble(Subsystem.Drivetrain.P));
    rightMain.config_kI(0, driveI.getDouble(Subsystem.Drivetrain.I));
    rightMain.config_kD(0, driveD.getDouble(Subsystem.Drivetrain.D));
    rightMain.config_kF(0, driveF.getDouble(Subsystem.Drivetrain.F));

    // Push the current potition data from the sensors to the NetworkTables
    leftDistance.setDouble(getLeftDistance());
    rightDistance.setDouble(getRightDistance());
  }

  /**
   * Arcade drive using voltage output to the motors [-12, 12]. Useful for
   * trajectory following
  */
  public void arcadeDriveVoltage() {

  }

  /** Arcade drive using percent output to the motor controllers */
  public void arcadeDrivePercentOutput(double throttle, double rotation) {
    driveDifferential.arcadeDrive(throttle, rotation);
  }

  /** Arcade drive using velocity control onboard the motor controllers */
  public void arcadeDriveVelocity(double throttle, double rotation) {
    // Check our input parameters
    if (   (throttle < -1.0 || 1.0 < throttle)
        || (rotation < -1.0 || 1.0 < rotation)) {
      throw new IllegalArgumentException("Throttle and rotation must be [-1,1]");
    }

    // Convert joystick input to left and right motor output.
    // Note that the ChassisSpeeds constructor vy arguement is 0 as the robot
    // can only drive forwards/backwards, not left/right
    DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(
        new ChassisSpeeds(
            throttle * Subsystem.Drivetrain.MAX_VELOCITY,
            0,
            rotation * Subsystem.Drivetrain.MAX_ROTATION));

    // Print out debugging information
    System.out.println("Got:    " + throttle * Subsystem.Drivetrain.MAX_VELOCITY
        + "    " + rotation * Subsystem.Drivetrain.MAX_ROTATION);
    System.out.println("Set:    "
        + (wheelSpeeds.leftMetersPerSecond * (1.0/10.0) * (4096.0/Chassis.WHEEL_CIRCUM))
        + "    " + (wheelSpeeds.rightMetersPerSecond * (1.0/10.0) * (4096.0/Chassis.WHEEL_CIRCUM)) );

    // Convert from m/s and set motor output to velocity in ticks/100ms
    leftMain.set(
      ControlMode.Velocity,
        wheelSpeeds.leftMetersPerSecond * (1.0/100.0) * (4096.0/Chassis.WHEEL_CIRCUM));
    rightMain.set(
        ControlMode.Velocity,
        wheelSpeeds.rightMetersPerSecond * (1.0/100.0) * (4096.0/Chassis.WHEEL_CIRCUM));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    syncNetworkTables();
    driveOdometry.update(
        getGyroRotation(),
        getLeftDistance(),
        getRightDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    syncNetworkTables();
  }
}
