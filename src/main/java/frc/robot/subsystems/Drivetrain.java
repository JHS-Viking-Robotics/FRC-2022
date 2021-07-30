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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonSRX leftMain;
  private final WPI_TalonSRX leftFollow;
  private final WPI_TalonSRX rightMain;
  private final WPI_TalonSRX rightFollow;
  private final DifferentialDrive driveDifferential;
  private final DifferentialDriveKinematics driveKinematics;

  private NetworkTableEntry driveP;
  private NetworkTableEntry driveI;
  private NetworkTableEntry driveD;
  private NetworkTableEntry driveF;
  private NetworkTableEntry driveLeftSetpoint;
  private NetworkTableEntry driveRightSetpoint;

  /**
   * Creates a new Drivetrain subsystem with 2 Talon and 2 Victor motor
   * controllers.
   */
  public Drivetrain() {
    // Initialize new Talon controllers and configure them
    leftMain = new WPI_TalonSRX(Subsystem.Drivetrain.LEFT_MAIN);
    rightMain = new WPI_TalonSRX(Subsystem.Drivetrain.RIGHT_MAIN);
    leftFollow = new WPI_TalonSRX(Subsystem.Drivetrain.LEFT_FOLLOW);
    rightFollow = new WPI_TalonSRX(Subsystem.Drivetrain.RIGHT_FOLLOW);
    configureTalons();
    
    // Configure differential drive, kinematics, and odometry
    driveDifferential = new DifferentialDrive(leftMain, rightMain);
    driveDifferential.setRightSideInverted(false);
    driveKinematics = new DifferentialDriveKinematics(Chassis.TRACK_WIDTH);

    // Configure Shuffleboard dashboard tab and NetworkTable entries
    configureShuffleboard();
  }

  /** Configures the Talon motor controllers and safety settings */
  private void configureTalons() {
    // Set Talon inversion, encoder phase and type, and set followers
    leftMain.setInverted(Subsystem.Drivetrain.LEFT_INVERTED);
    leftMain.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    leftMain.setSensorPhase(true);

    rightMain.setInverted(Subsystem.Drivetrain.RIGHT_INVERTED);
    rightMain.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightMain.setSensorPhase(true);

    leftFollow.setInverted(Subsystem.Drivetrain.LEFT_INVERTED);
    leftFollow.follow(leftMain);

    rightFollow.setInverted(Subsystem.Drivetrain.RIGHT_INVERTED);
    rightFollow.follow(rightMain);

    // Set Talon safety parameters
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
  }

  /** Get the left encoder total distance travelled in meters*/
  public double getLeftDistance() {
    // Get the quadrature encoder position in ticks (4096 ticks/rotation)    
    // Convert from raw ticks and return distance in meters
    return leftMain.getSelectedSensorPosition() * (Chassis.WHEEL_CIRCUM / 4096);
  }

  /** Get the right encoder total distance travelled in meters*/
  public double getRightDistance() {
    // Get the quadrature encoder position in ticks (4096 ticks/rotation)    
    // Convert from raw ticks and return distance in meters
    return rightMain.getSelectedSensorPosition() * (Chassis.WHEEL_CIRCUM / 4096);
  }

  /** Get the average total distance travelled in meters from both encoders */
  public double getTotalDistance() {
    // Average the left and right encoder distances
    return (getRightDistance() + getLeftDistance()) / 2;
  }

  /** Reset the distance travelled for both encoders */
  public void resetDistance() {
    leftMain.setSelectedSensorPosition(0);
    rightMain.setSelectedSensorPosition(0);
  }

  /**
   * Update the PIDF configuration for both encoders from the NetworkTables values
   * configured on the Shuffleboard
   * 
   * @see Drivetrain#setPIDF(double, double, double, double)
   */
  public void setPIDF() {
    // Configure the Talon closed-loop PID values from the Shuffleboard
    // NetworkTables values
    leftMain.config_kP(0, driveP.getDouble(Subsystem.Drivetrain.P));
    leftMain.config_kI(0, driveI.getDouble(Subsystem.Drivetrain.I));
    leftMain.config_kD(0, driveD.getDouble(Subsystem.Drivetrain.D));
    leftMain.config_kF(0, driveF.getDouble(Subsystem.Drivetrain.F));
    rightMain.config_kP(0, driveP.getDouble(Subsystem.Drivetrain.P));
    rightMain.config_kI(0, driveI.getDouble(Subsystem.Drivetrain.I));
    rightMain.config_kD(0, driveD.getDouble(Subsystem.Drivetrain.D));
    rightMain.config_kF(0, driveF.getDouble(Subsystem.Drivetrain.F));
  }

  /** Updates the PIDF configuration for both encoders by writing to the
   * NetworkTables entry. Note that the values will only be applied after the
   * next PeriodicTask cycle is done
   * 
   * @param P constant
   * @param I constant
   * @param D constant
   * @param F constant
  */
  public void setPIDF(double P, double I, double D, double F) {
    // Manually update the PIDF values in NetworkTables
    boolean a = !driveP.setDouble(P);
    boolean b = !driveI.setDouble(I);
    boolean c = !driveD.setDouble(D);
    boolean d = !driveF.setDouble(F);

    // Print an error message if any of the updates failed
    String errorMsg = "Error in Drivetrain.setPIDF(double,double,double,double):"
    + "Entry already exists with different type";
    if (a||b||c||d) {System.out.println(errorMsg);}
  }

  /** Arcade drive using percent output to the motor controllers */
  public void arcadeDrivePercentOutput(double throttle, double rotation) {
    driveDifferential.arcadeDrive(throttle, rotation);
  }

  /** Arcade drive using velocity control onboard the motor controllers */
  public void arcadeDriveVelocity(double throttle, double rotation) {
    // Convert joystick input to left and right motor output.
    // NOTE: The ChassisSpeeds constructor vy arguement is 0 as the robot can
    //       only drive forwards/backwards
    DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(
        new ChassisSpeeds(
            throttle * Subsystem.Drivetrain.MAX_VELOCITY,
            0,
            rotation * Subsystem.Drivetrain.MAX_ROTATION));

    System.out.println("Got:    " + throttle * Subsystem.Drivetrain.MAX_VELOCITY + "    " + rotation * Subsystem.Drivetrain.MAX_ROTATION);
    // Convert m/s and set motor output to velocity in ticks/100ms
    System.out.println("Set:    " + 
        (wheelSpeeds.leftMetersPerSecond * (1.0/10.0) * (4096.0/Chassis.WHEEL_CIRCUM))
        + "    "
        + (wheelSpeeds.rightMetersPerSecond * (1.0/10.0) * (4096.0/Chassis.WHEEL_CIRCUM)) );
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
    setPIDF();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    setPIDF();
  }
}
