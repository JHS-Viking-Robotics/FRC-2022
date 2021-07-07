// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonSRX leftMain;
  private final WPI_TalonSRX leftFollow;
  private final WPI_TalonSRX rightMain;
  private final WPI_TalonSRX rightFollow;

  private final DifferentialDrive diffDrivetrain;

  /** Creates a new Drivetrain subsystem with 2 Talon and 2 Victor motor controllers. */
  public Drivetrain() {

    // Initialize new Talon controllers and set followers
    leftMain = new WPI_TalonSRX(Constants.Talon.Drivetrain.LEFT_MAIN);
    rightMain = new WPI_TalonSRX(Constants.Talon.Drivetrain.RIGHT_MAIN);
    leftFollow = new WPI_TalonSRX(Constants.Talon.Drivetrain.LEFT_FOLLOW);
    rightFollow = new WPI_TalonSRX(Constants.Talon.Drivetrain.RIGHT_FOLLOW);
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

    // Initialize new differential drivetrain with the new left and right controllers
    diffDrivetrain = new DifferentialDrive(leftMain, rightMain);

  }
  
  /** Arcade drive using percent output to the motor controllers */
  public void arcadeDrivePercentOutput(double throttle, double rotation) {
    diffDrivetrain.arcadeDrive(throttle, rotation);
  }

/*
  // Arcade drive using velocity control onboard the motor controllers
  public void arcadeDriveVelocity() {
  
    // TODO: Add velocity controlled drive (requires encoders to be set up)
  
  }
*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
