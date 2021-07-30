// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.Talon;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

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

    // Initialize new Talon controllers and configure them
    liftController = new WPI_TalonSRX(Talon.Hopper.LIFT_ID);
    intakeController = new WPI_TalonSRX(Talon.Hopper.INTAKE_ID);
    configureTalons();

    // Configure Shuffleboard dashboard tab and NetworkTable entries
    configureShuffleboard();
  }

  /** Configures the Talon motor controllers and safety settings */
  private void configureTalons() {
    // Set Talon inversion, sensor phase, and sensor type
    liftController.setInverted(Talon.Hopper.LIFT_CONTROLLER_INVERTED);
    liftController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    liftController.setSensorPhase(Talon.Hopper.LIFT_SENSOR_INVERTED);
    liftController.setSelectedSensorPosition(0.0);

    intakeController.setInverted(Talon.Hopper.INTAKE_CONTROLLER_INVERTED);

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
  }

  /** Configures the Shuffleboard dashboard "Hopper" tab */
  private void configureShuffleboard() {
    // Create references to Hopper tab and PID layout
    ShuffleboardTab shuffleHopperTab = Shuffleboard.getTab("Hopper");
    ShuffleboardLayout shufflePIDLayout = shuffleHopperTab
        .getLayout("Lift PID", BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "LEFT"))
        .withPosition(0, 0)
        .withSize(1, 2);

    // Configure PID list widget, and set default values from Constants
    liftP = shufflePIDLayout
        .add("P", Talon.Hopper.LIFT_P)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    liftI = shufflePIDLayout
        .add("I", Talon.Hopper.LIFT_I)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    liftD = shufflePIDLayout
        .add("D", Talon.Hopper.LIFT_D)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    liftF = shufflePIDLayout
        .add("F", Talon.Hopper.LIFT_F)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    liftSetpoint = shuffleHopperTab
        .add("Lift Setpoint", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("Min", 0, "Max", 800))
        .withSize(4, 1)
        .withPosition(1, 0)
        .getEntry();
  }

  /** Gets the current Lift position in sensor ticks */
  public double getLiftHeightTicks() {
    return liftController.getSelectedSensorPosition();
  }

  /** Sets a new Lift position in sensor ticks */
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
   * Updates the PIDF configuration for the Lift encoder from the NetworkTables
   * values configured on the Shuffleboard
   * 
   * @see Hopper#setLiftPIDF(double, double, double, double)
   */
  public void setLiftPIDF() {
    // Configure the Talon closed-loop PID values from the Shuffleboard
    // NetworkTables values
    liftController.config_kP(0, liftP.getDouble(Talon.Hopper.LIFT_P));
    liftController.config_kI(0, liftI.getDouble(Talon.Hopper.LIFT_I));
    liftController.config_kD(0, liftD.getDouble(Talon.Hopper.LIFT_D));
    liftController.config_kF(0, liftF.getDouble(Talon.Hopper.LIFT_F));
  }

  /** Sets the PIDF configuration for the lift encoder by writing to the
   * NetworkTables entry. Note that the values will only be applied after the
   * next PeriodicTask cycle is done
   * 
   * @param P constant
   * @param I constant
   * @param D constant
   * @param F constant
  */
  public void setLiftPIDF(double P, double I, double D, double F) {
    // Manually update the PIDF values in NetworkTables
    boolean a = !liftP.setDouble(P);
    boolean b = !liftI.setDouble(I);
    boolean c = !liftD.setDouble(D);
    boolean d = !liftF.setDouble(F);
  
    // Print an error message if any of the updates failed
    String errorMsg = "Error in Hopper.setLiftPIDF(double,double,double,double):"
    + "Entry already exists with different type";
    if (a||b||c||d) {System.out.println(errorMsg);}
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setLiftPIDF();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    setLiftPIDF();
  }}
