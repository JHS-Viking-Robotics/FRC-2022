// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.Subsystem;

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
  private NetworkTableEntry liftPosition;
  private NetworkTableEntry liftUpSetpoint;
  private NetworkTableEntry liftDispenseSetpoint;
  private NetworkTableEntry liftDownSetpoint;
  private NetworkTableEntry intakeInSpeed;
  private NetworkTableEntry intakeOutSpeed;
  private NetworkTableEntry intakeHoldCurrent;

  public enum Intake {
    IN,
    OUT,
    HOLD,
    NEUTRAL;
  }

  public enum Lift {
    UP,
    DISPENSE,
    DOWN;
  }

  /**
   * Creates a new Hopper subsystem with a Lift arm controlled with a Talon SRX
   * and mag encoder, and a roller intake controlled by a Victor SPX.
   */
  public Hopper() {

    // Initialize new Talon controllers and configure them
    liftController = new WPI_TalonSRX(Subsystem.Hopper.LIFT_ID);
    intakeController = new WPI_TalonSRX(Subsystem.Hopper.INTAKE_ID);
    configureTalons();

    // Configure Shuffleboard dashboard tab and NetworkTable entries
    configureShuffleboard();
  }

  /** Configures the Talon motor controllers and safety settings */
  private void configureTalons() {
    // Set Talon inversion, sensor phase, and sensor type
    liftController.setInverted(Subsystem.Hopper.LIFT_CONTROLLER_INVERTED);
    liftController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    liftController.setSensorPhase(Subsystem.Hopper.LIFT_SENSOR_INVERTED);
    liftController.setSelectedSensorPosition(0.0);

    intakeController.setInverted(Subsystem.Hopper.INTAKE_CONTROLLER_INVERTED);

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
    ShuffleboardLayout shuffleLiftPIDLayout = shuffleHopperTab
        .getLayout("Lift PID", BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "LEFT"))
        .withPosition(0, 0)
        .withSize(1, 2);
    ShuffleboardLayout shuffleLiftSetpointsLayout = shuffleHopperTab
        .getLayout("Lift Setpoints", BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "TOP"))
        .withPosition(1, 0)
        .withSize(1, 2);
    ShuffleboardLayout shuffleIntakeLayout = shuffleHopperTab
        .getLayout("Intake Modes", BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "TOP"))
        .withPosition(6, 0)
        .withSize(1, 2);

    // Configure Lift PID list, and set default values from frc.robot.Constants
    liftP = shuffleLiftPIDLayout
        .add("P", Subsystem.Hopper.LIFT_P)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    liftI = shuffleLiftPIDLayout
        .add("I", Subsystem.Hopper.LIFT_I)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    liftD = shuffleLiftPIDLayout
        .add("D", Subsystem.Hopper.LIFT_D)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    liftF = shuffleLiftPIDLayout
        .add("F", Subsystem.Hopper.LIFT_F)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    liftPosition = shuffleHopperTab
        .add("Lift Position", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("Min", 0, "Max", 800))
        .withSize(4, 1)
        .withPosition(2, 0)
        .getEntry();
   liftSetpoint = shuffleHopperTab
        .add("Lift Setpoint", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("Min", 0, "Max", 800))
        .withSize(4, 1)
        .withPosition(2, 1)
        .getEntry();
    
    // Configure Lift positions and set default values from frc.robot.Constants
    liftUpSetpoint = shuffleLiftSetpointsLayout
        .add("Up", Subsystem.Hopper.LIFT_UP)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    liftDispenseSetpoint = shuffleLiftSetpointsLayout
        .add("Dispense", Subsystem.Hopper.LIFT_DISPENSE)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    liftDownSetpoint = shuffleLiftSetpointsLayout
        .add("Down", Subsystem.Hopper.LIFT_DOWN)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    // Configure Intake Speeds and set default values from frc.robot.Constants
    intakeInSpeed = shuffleIntakeLayout
        .add("In speed", Subsystem.Hopper.INTAKE_IN)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    intakeOutSpeed = shuffleIntakeLayout
        .add("Out speed", Subsystem.Hopper.INTAKE_OUT)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    intakeHoldCurrent = shuffleIntakeLayout
        .add("Hold current", Subsystem.Hopper.INTAKE_HOLD)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
  }

  /** Gets the current Lift position in sensor ticks */
  public double getLiftPositionTicks() {
    return liftController.getSelectedSensorPosition();
  }

  /** Gets the current Lift position in meters <p>WARNING: Not yet implemented */
  public double getLiftPositionMeters() {
    // TODO: Implement issue #18
    System.out.println("Error in frc.robot.subsystems.Hopper.getLiftPositionMeters(): Feature not yet implemented");
    return -1234.5678;
  }
  
  /** Gets the current Lift error */
  public double getLiftPositionError() {
    return liftController.getClosedLoopError();
  }
  
  /**
   * Gets the current Lift setpoint in sensor ticks.
   * 
   * <p> Returns -123456.7 if there is an error fetching the value.
   */
  public double getLiftSetpointTicks() {
    return liftSetpoint.getDouble(-123456.7);
  }

  /**
   * Resets the sensor position to 0
   * 
   * @see #resetLiftSensorPosition(double)
   */
  public void resetLiftSensorPosition() {
    resetLiftSensorPosition(0.0);
  }
  
  /** Resets the sensor position to the given position */
  public void resetLiftSensorPosition(double sensorPosition) {
    liftController.setSelectedSensorPosition(sensorPosition);
  }

  /**
   * Resets the setpoint value to the default in
   * {@link frc.robot.Constants Constants.java}
   * 
   * {@see resetLiftSetpoint(Lift, double)}
   */
  public void resetLiftSetpoint(Lift setpoint) {
    switch (setpoint) {
        case UP: {
            liftUpSetpoint.setDouble(Subsystem.Hopper.LIFT_UP);
            break;
        }
        case DISPENSE: {
            liftDispenseSetpoint.setDouble(Subsystem.Hopper.LIFT_DISPENSE);
            break;
        }
        case DOWN: {
            liftDownSetpoint.setDouble(Subsystem.Hopper.LIFT_DOWN);
            break;
        }
        default: {
            throw new IllegalArgumentException("Error in frc.robot.subsystems.Hopper.resetLiftSetpoint(Lift): Cannot handle " + setpoint);
        }
    }
  }

  /** Resets the setpoint value to the specified value */
  public void resetLiftSetpoint(Lift setpoint, double value) {
    switch (setpoint) {
        case UP: {
            liftUpSetpoint.setDouble(value);
            break;
        }
        case DISPENSE: {
            liftDispenseSetpoint.setDouble(value);
            break;
        }
        case DOWN: {
            liftDownSetpoint.setDouble(value);
            break;
        }
        default: {
            throw new IllegalArgumentException("Error in frc.robot.subsystems.Hopper.resetLiftSetpoint(Lift, double): Cannot handle " + setpoint);
        }
    }
  }

  /** Sets the Lift to the specified operating position */
  public void setLiftSetpoint(Lift setpoint) {
    // Use Lift enum to set Lift output and mode
    switch (setpoint) {
        case UP: {
            liftController.set(
                ControlMode.Position,
                liftUpSetpoint.getDouble(Subsystem.Hopper.LIFT_UP));
            break;
        }
        case DISPENSE: {
            liftController.set(
                ControlMode.Position,
                liftDispenseSetpoint.getDouble(Subsystem.Hopper.LIFT_DISPENSE));
            break;
        }
        case DOWN: {
            liftController.set(
                ControlMode.Position,
                liftDownSetpoint.getDouble(Subsystem.Hopper.LIFT_DOWN));
            break;
        }
        default: {
            liftController.set(
                ControlMode.Disabled,
                0);
            throw new IllegalArgumentException("Error in frc.robot.subsystems.Hopper.setLiftSetpoint(Lift): Cannot handle " + setpoint);
        }
    }
  }

  /**
   * Sets the Lift setpoint to a specified position in ticks
   */
  public void setLiftSetpointTicks(double setpoint) {
    // Update the NetworkTables entry and set the new setpoint on the controller
    liftSetpoint.setDouble(setpoint);
    liftController.set(ControlMode.Position, setpoint);
  }

  /**
   * Sets the Lift setpoint from the NetworkTables entry
   * 
   * @see #setLiftSetpointTicks(double)
  */
  public void setLiftSetpointTicks() {
    // Set the new setpoint on the controller from NetworkTables
    liftController.set(ControlMode.Position, liftSetpoint.getDouble(0.0));
  }

  /**
   * Sets the Lift setpoint to the specified height in meters
   * 
   * <p> WARNING: Not yet implemented
   */
  public void setLiftSetpointMeters(double setpoint) {
    // TODO: Implement set lift height in meters, issue #18
    System.out.println("Error in Hopper.setLiftSetpointMeters(double): Not yet implemented");
  }

  /** Sets the Intake to the specified operating mode */
  public void setIntake(Intake mode) {
    // Use Intake enum to set Intake output and mode
    switch (mode) {
        case IN: {
            intakeController.set(
                ControlMode.PercentOutput,
                intakeInSpeed.getDouble(Subsystem.Hopper.INTAKE_IN));
            break;
        }
        case OUT: {
            intakeController.set(
                ControlMode.PercentOutput,
                intakeOutSpeed.getDouble(-Subsystem.Hopper.INTAKE_OUT));
            break;
        }
        case HOLD: {
            // TODO: Implement current control, issue #31
            System.out.println("Error in frc.robot.subsystems.Hopper.setIntake(Intake): Feature not yet implemented");
            intakeController.set(
                ControlMode.Current,
                intakeHoldCurrent.getDouble(Subsystem.Hopper.INTAKE_HOLD));
            break;
        }
        case NEUTRAL: {
            intakeController.set(
                ControlMode.PercentOutput,
                0.0);
            break;
        }
        default: {
            intakeController.set(
                ControlMode.Disabled,
                0);
        throw new IllegalArgumentException("Error in frc.robot.subsystems.Hopper.setIntake(Intake): Cannot handle " + mode);
        }
    }
  }

  /**
   * Sets all Hopper motors to neutral. Useful for an emergency situation
   * to quickly disable the Hopper
   */
  public void setAllNeutral() {
    liftController.set(ControlMode.Disabled, 0);
    intakeController.set(ControlMode.Disabled, 0);
  }

  /** 
   * Synchronizes member variables and motor controller values with those
   * in NetworkTables
   */
  public void syncNetworkTables() {
    // Pull the current Lift PIDF constants and apply them
    liftController.config_kP(0, liftP.getDouble(Subsystem.Hopper.LIFT_P));
    liftController.config_kI(0, liftI.getDouble(Subsystem.Hopper.LIFT_I));
    liftController.config_kD(0, liftD.getDouble(Subsystem.Hopper.LIFT_D));
    liftController.config_kF(0, liftF.getDouble(Subsystem.Hopper.LIFT_F));
    
    // Push the current position from the Lift sensor
    liftPosition.setDouble(getLiftPositionTicks());
  }

  /**
   * Sets the PIDF configuration for the lift encoder by writing to the
   * NetworkTables entry.
   * 
   * <p>Note that the values will only be applied after the
   * next {@link #syncNetworkTables() synchronization cycle} is ran
   * during the next PeriodicTask
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
    syncNetworkTables();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    syncNetworkTables();
  }
}
