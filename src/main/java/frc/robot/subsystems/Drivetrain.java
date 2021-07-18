// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.Chassis;
import frc.robot.Constants.Talon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonSRX leftMain;
  private final WPI_TalonSRX leftFollow;
  private final WPI_TalonSRX rightMain;
  private final WPI_TalonSRX rightFollow;
  private final DifferentialDrive driveDifferential;
  private final DifferentialDriveKinematics driveKinematics;
  
  private ShuffleboardTab shuffleboardTab;
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

    // Initialize new Talon controllers and set followers
    leftMain = new WPI_TalonSRX(Talon.Drivetrain.LEFT_MAIN);
    rightMain = new WPI_TalonSRX(Talon.Drivetrain.RIGHT_MAIN);
    leftFollow = new WPI_TalonSRX(Talon.Drivetrain.LEFT_FOLLOW);
    rightFollow = new WPI_TalonSRX(Talon.Drivetrain.RIGHT_FOLLOW);
    leftMain.setInverted(Talon.Drivetrain.LEFT_INVERTED);
    rightMain.setInverted(Talon.Drivetrain.RIGHT_INVERTED);
    leftFollow.setInverted(Talon.Drivetrain.LEFT_INVERTED);
    rightFollow.setInverted(Talon.Drivetrain.RIGHT_INVERTED);
    leftFollow.follow(leftMain);
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

    // Configure encoders and PID settings
    setPID(
        Talon.Drivetrain.P,
        Talon.Drivetrain.I,
        Talon.Drivetrain.D,
        Talon.Drivetrain.F);

    // Configure differential drive, kinematics, and odometry
    driveDifferential = new DifferentialDrive(leftMain, rightMain);
    driveKinematics = new DifferentialDriveKinematics(Chassis.TRACK_WIDTH);

    // Configure Shuffleboard dashboard tab
    shuffleboardTab = Shuffleboard.getTab("Drivetrain");

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

  /** Update the PIDF configuration for both encoders from the Shuffleboard Net Tables values */
  public void setPID() {
    // Configure the Talon closed-loop PID values from the dashboard
    setPID(
      driveP.getDouble(0),
      driveI.getDouble(0),
      driveD.getDouble(0),
      driveF.getDouble(0));
  }

  /** Update the PIDF configuration for both encoders manually
   * 
   * @param P constant
   * @param I constant
   * @param D constant
   * @param F constant
  */
  public void setPID(double P, double I, double D, double F) {
    // Configure the Talon closed-loop PID values
    leftMain.config_kP(0, P);
    leftMain.config_kI(0, I);
    leftMain.config_kD(0, D);
    leftMain.config_kF(0, F);
    rightMain.config_kP(0, P);
    rightMain.config_kI(0, I);
    rightMain.config_kD(0, D);
    rightMain.config_kF(0, F);

    // Push the new values to the Shuffleboard
    driveP.setDouble(P);
    driveI.setDouble(I);
    driveD.setDouble(D);
    driveF.setDouble(F);
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
            throttle * Talon.Drivetrain.MAX_VELOCITY,
            0,
            rotation * Talon.Drivetrain.MAX_ROTATION));

    // Convert m/s and set motor output to velocity in ticks/100ms
    leftMain.set(
        ControlMode.Velocity,
        wheelSpeeds.leftMetersPerSecond * (1/10) * (4096/Chassis.WHEEL_CIRCUM));
    rightMain.set(
        ControlMode.Velocity,
        wheelSpeeds.rightMetersPerSecond * (1/10) * (4096/Chassis.WHEEL_CIRCUM));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setPID();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
