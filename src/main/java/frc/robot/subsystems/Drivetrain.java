// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.Talon;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonSRX leftMain;
  private final WPI_TalonSRX leftFollow;
  private final WPI_TalonSRX rightMain;
  private final WPI_TalonSRX rightFollow;

  private final DifferentialDrive diffDrivetrain;

  /** Creates a new Drivetrain subsystem with 2 Talon and 2 Victor motor controllers. */
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

    // Set parameters, PID values, and safety config
    leftMain.configFactoryDefault();
    rightMain.configFactoryDefault();
    leftMain.configPeakCurrentLimit(0);
    rightMain.configPeakCurrentLimit(0);
    leftMain.configContinuousCurrentLimit(35);
    rightMain.configContinuousCurrentLimit(35);
    leftMain.configPeakCurrentDuration(100);
    rightMain.configPeakCurrentDuration(100);
    leftMain.setSafetyEnabled(true);
    rightMain.setSafetyEnabled(true);


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
  
  /** Arcade drive using percent output to the motor controllers */
  public void arcadeDrivePercentOutput(double throttle, double rotation) {
    // Configure left and right talons as WPI DifferentialDrive type, and drive with arcade drive
    diffDrivetrain = new DifferentialDrive(leftMain, rightMain);
    diffDrivetrain.arcadeDrive(throttle, rotation);
  }

  /** Arcade drive using velocity control onboard the motor controllers */
  public void arcadeDriveVelocity(double throttle, double rotation) {
    /*
      Convert inputs to actual motor velocity values in arcade style. Input throttle
      is only called throttle to parallel DifferentialDrive.arcadeDrive(), but is
      actually passed as a double between 0.0 and 1.0 from a joystick axis. Therefore,
      we need to do a conversion like the one described here:
      https://electronics.stackexchange.com/questions/19669/algorithm-for-mixing-2-axis-analog-input-to-control-a-differential-motor-drive

      Because this.arcadeDriveVelocity() is meant to parallel this.arcadeDrivePercentOutput(),
      which utilizes the WPI Lib DifferentialDrive.arcadeDrive(), we will use their
      conversion so the performance is similar.
    */
    double xSpeed = throttle;
    double zRotation = rotation;
    double leftOutput;
    double rightOutput;
    // double maxVelocityFPS = 1; // Max velocity in feet / s
    double maxVelocityTicks = 100; // Max velocity in encoder ticks / 100ms

    // Convert values from analog input to motor outputs
    // TODO: clean up this math and/or put it in another method
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    if (Math.abs(xSpeed) > 0.2) {
      if (xSpeed > 0.0) {
        xSpeed = (xSpeed - 0.2) / (1.0 - 0.2);
      } else {
        xSpeed = (xSpeed + 0.2) / (1.0 - 0.2);
      }
    } else {
      xSpeed = 0.0;
    }
    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);

    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    if (Math.abs(zRotation) > 0.2) {
      if (zRotation > 0.0) {
        zRotation = (zRotation - 0.2) / (1.0 - 0.2);
      } else {
        zRotation = (zRotation + 0.2) / (1.0 - 0.2);
      }
    } else {
      xSpeed = 0.0;
    }
    zRotation = Math.copySign(zRotation * zRotation, zRotation);

    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftOutput = 1.0;
        rightOutput = xSpeed - zRotation;
      } else {
        leftOutput = xSpeed + zRotation;
        rightOutput = 1.0;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftOutput = xSpeed + zRotation;
        rightOutput = 1.0;
      } else {
        leftOutput = 1.0;
        rightOutput = xSpeed - zRotation;
      }
    }
    
    leftOutput = MathUtil.clamp(leftOutput, -1.0, 1.0) * maxVelocityTicks;
    rightOutput = MathUtil.clamp(rightOutput, -1.0, 1.0) * maxVelocityTicks;

    leftMain.set(ControlMode.Velocity, leftOutput);
    rightMain.set(ControlMode.Velocity, rightOutput);
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
