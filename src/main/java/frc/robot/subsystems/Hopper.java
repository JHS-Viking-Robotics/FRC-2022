// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.Talon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

  enum Height {
    UP,
    DOWN,
    DISPENSE
  }

  enum Intake {
    IN,
    OUT,
    NEUTRAL,
    HOLD
  }

  private final WPI_TalonSRX liftController;
  private final WPI_TalonSRX intakeController;

  private NetworkTableEntry liftP;
  private NetworkTableEntry liftI;
  private NetworkTableEntry liftD;
  private NetworkTableEntry liftF;
  private NetworkTableEntry liftSetpoint;

  /**
   * Creates a new Hopper subsystem with a Lift arm controlled with a Talon SRX
   * and mag encoder, and a roller intake controlled by a Victor SPX.
   */
  public Hopper() {

    // Initialize motor controllers
    liftController = new WPI_TalonSRX(Talon.Hopper.LIFT_ID);
    intakeController = new WPI_TalonSRX(Talon.Hopper.INTAKE_ID);

    // Set Talon safety parameters
    liftController.configFactoryDefault();
    liftController.configPeakCurrentLimit(0);
    liftController.configContinuousCurrentLimit(35);
    liftController.configPeakCurrentDuration(100);
    liftController.enableCurrentLimit(true);
    liftController.setSafetyEnabled(true);
    intakeController.configFactoryDefault();
    intakeController.configPeakCurrentLimit(0);
    intakeController.configContinuousCurrentLimit(35);
    intakeController.configPeakCurrentDuration(100);
    intakeController.enableCurrentLimit(true);
    intakeController.setSafetyEnabled(true);

    // Configure encoders and PID settings
    setLiftPID(
        Talon.Hopper.LIFT_P,
        Talon.Hopper.LIFT_I,
        Talon.Hopper.LIFT_D,
        Talon.Hopper.LIFT_F);
  }

  /** Get the current Lift position in sensor ticks */
  public double getLiftHeightTicks() {
    return liftController.getSelectedSensorPosition();
  }

  /** Set a new Lift position in sensor ticks */
  public void setLiftHeightTicks(double position) {
    liftController.set(ControlMode.Position, position);
  }

  public void setIntakeIn() {
    intakeController.set(ControlMode.PercentOutput, .50);
  }

  public void setIntakeOut() {
    intakeController.set(ControlMode.PercentOutput, -.50);
  }

  public void setIntakeNeutral() {
    intakeController.set(ControlMode.PercentOutput, 0.00);
  }
  
  /**
   * Update the PIDF configuration for the Lift encoder from the Shuffleboard
   * Net Tables values 
   */
  public void setLiftPID() {
    // Configure the Talon closed-loop PID values from the dashboard
    setLiftPID(
      liftP.getDouble(0),
      liftI.getDouble(0),
      liftD.getDouble(0),
      liftF.getDouble(0));
  }

  /** Update the PIDF configuration for the Lift encoder manually
   * 
   * @param P constant
   * @param I constant
   * @param D constant
   * @param F constant
   */
  public void setLiftPID(double P, double I, double D, double F) {
    // Configure the Talon closed-loop PID values
    liftController.config_kP(0, P);
    liftController.config_kI(0, I);
    liftController.config_kD(0, D);
    liftController.config_kF(0, F);

    // Push the new values to the Shuffleboard
    liftP.setDouble(P);
    liftI.setDouble(I);
    liftD.setDouble(D);
    liftF.setDouble(F);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
