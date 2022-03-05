// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Subsystem.Shooter.*;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final CANSparkMax topLeft;              // Top left shooter motor
  private final CANSparkMax topRight;             // Top right shooter motor
  private final CANSparkMax bottomLeft;           // Bottom left shooter motor
  private final CANSparkMax bottomRight;          // Bottom right shooter motor
  private final DoubleSolenoid trigger;           // Piston to push balls into firing position

  private NetworkTableEntry topLeftVelocity;      // NetworkTables speedometer for top left sensor
  private NetworkTableEntry topRightVelocity;     // NetworkTables speedometer for top right sensor
  private NetworkTableEntry bottomLeftVelocity;   // NetworkTables speedometer for bottom left sensor
  private NetworkTableEntry bottomRightVelocity;  // NetworkTables speedometer for bottom right sensor
  private NetworkTableEntry shooterSpeed;         // NetworkTables controller for motor speed

  /** Creates a new Shooter. */
  public Shooter() {
    // Connect to the motor controllers and piston
    topLeft = new CANSparkMax(TOP_LEFT_ID, MotorType.kBrushless);
    topRight = new CANSparkMax(TOP_RIGHT_ID, MotorType.kBrushless);
    bottomLeft = new CANSparkMax(BOTTOM_LEFT_ID, MotorType.kBrushless);
    bottomRight = new CANSparkMax(BOTTOM_RIGHT_ID, MotorType.kBrushless);
    trigger = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    // Configure the motors. Positive direction should be out
    topLeft.restoreFactoryDefaults();
    topRight.restoreFactoryDefaults();
    bottomLeft.restoreFactoryDefaults();
    bottomRight.restoreFactoryDefaults();
    topLeft.setInverted(TOP_LEFT_INVERTED);
    topRight.setInverted(TOP_RIGHT_INVERTED);
    bottomLeft.setInverted(BOTTOM_LEFT_INVERTED);
    bottomRight.setInverted(BOTTOM_RIGHT_INVERTED);

    // Piston should start retracted
    trigger.set(kReverse);

    // Configure the shuffleboard
    configureShuffleboard();
  }

  /** Configures the Shuffleboard dashboard "Drivetrain" tab */
  private void configureShuffleboard() {
    // Create references to Drivetrain tab and its various layouts
    ShuffleboardTab shuffleShooterTab = Shuffleboard.getTab("Shooter");

    // Add piston object to tab
    shuffleShooterTab
        .add("Trigger Piston", trigger)
        .withPosition(0, 0)
        .withSize(2, 2);

    // Configure speedometers for each motor
    topLeftVelocity = shuffleShooterTab
        .add("Top Left Velocity", 0.0)
        .withWidget(kTextView)
        .withPosition(2, 0)
        .withSize(1, 1)
        .getEntry();
    topRightVelocity = shuffleShooterTab
        .add("Top Right Velocity", 0.0)
        .withWidget(kTextView)
        .withPosition(3, 0)
        .withSize(1, 1)
        .getEntry();
    bottomLeftVelocity = shuffleShooterTab
        .add("Bottom Left Velocity", 0.0)
        .withWidget(kTextView)
        .withPosition(2, 1)
        .withSize(1, 1)
        .getEntry();
    bottomRightVelocity = shuffleShooterTab
        .add("Bottom Right Velocity", 0.0)
        .withWidget(kTextView)
        .withPosition(3, 1)
        .withSize(1, 1)
        .getEntry();
    
    // Configure speed slider
    shooterSpeed = shuffleShooterTab
        .add("Shooter Speed", 0.0)
        .withWidget(kNumberSlider)
        .withProperties(
            Map.of(
                "Min", 0,
                "Max", 1))
        .withSize(2, 1)
        .withPosition(0, 2)
        .getEntry();
  }

  /*
   * Toggle the firing motors on or off. Note that this may take a second to
   * take effect as the motors spin up
   */
  public void toggleMotors(boolean active) {
    double output = active ? shooterSpeed.getDouble(0.0) : 0;
    topLeft.set(output);
    topRight.set(output);
    bottomLeft.set(output);
    bottomRight.set(output);
  }

  /*
   * Toggle the firing motors on or off. Note that this may take a second to
   * take effect as the motors spin up
   */
  public void toggleMotors() {
    double output = (topLeft.getEncoder().getVelocity() > 1) ? 0 : shooterSpeed.getDouble(0.0);
    topLeft.set(output);
    topRight.set(output);
    bottomLeft.set(output);
    bottomRight.set(output);
  }

  /*
   * Toggle the firing trigger in or out
   */
  public void toggleTrigger(boolean active) {
    if (active) { trigger.set(kForward); }
    else { trigger.set(kReverse); }
  }

  /*
   * Toggle the firing trigger in or out
   */
  public void toggleTrigger() {
    trigger.toggle();
  }

  /**
   * Update the speedometers in NetworkTables
   */
  public void syncNetworkTables() {
    // Push Drivetrain PIDF constants from NetworkTables to the controllers
    topLeftVelocity.setDouble(topLeft.getEncoder().getVelocity());
    topRightVelocity.setDouble(topRight.getEncoder().getVelocity());
    bottomLeftVelocity.setDouble(bottomLeft.getEncoder().getVelocity());
    bottomRightVelocity.setDouble(bottomRight.getEncoder().getVelocity());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    syncNetworkTables();
  }
}
