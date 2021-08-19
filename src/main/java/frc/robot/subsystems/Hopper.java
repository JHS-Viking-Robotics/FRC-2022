// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.Subsystem;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

  private final WPI_TalonSRX liftController;
  private final WPI_VictorSPX intakeController;

  private NetworkTableEntry liftP;
  private NetworkTableEntry liftI;
  private NetworkTableEntry liftD;
  private NetworkTableEntry liftF;
  private NetworkTableEntry liftSetpoint;
  private NetworkTableEntry liftPosition;
  private NetworkTableEntry liftUpSetpoint;
  private NetworkTableEntry liftDispenseSetpoint;
  private NetworkTableEntry liftDownSetpoint;
  private NetworkTableEntry intakeSetpoint;
  private NetworkTableEntry intakeInSpeed;
  private NetworkTableEntry intakeOutSpeed;
  private NetworkTableEntry intakeHoldCurrent;
  private NetworkTableEntry subsystemEnabled;

  public enum Intake {
    IN,
    OUT,
    HOLD,
    NEUTRAL;
  }

  public enum Lift {
    UP,
    DISPENSE,
    DOWN,
    NEUTRAL;
  }

  /**
   * Creates a new Hopper subsystem with a Lift arm controlled with a Talon SRX
   * and mag encoder, and a roller intake controlled by a Victor SPX.
   */
  public Hopper() {

    // Initialize new Talon controllers and configure them
    liftController = new WPI_TalonSRX(Subsystem.Hopper.LIFT_ID);
    intakeController = new WPI_VictorSPX(Subsystem.Hopper.INTAKE_ID);
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
    liftController.set(ControlMode.Disabled, 0.0);

    intakeController.setInverted(Subsystem.Hopper.INTAKE_CONTROLLER_INVERTED);
    intakeController.set(ControlMode.Disabled, 0.0);

    // Set Talon safety parameters
    liftController.configFactoryDefault();
    liftController.configPeakCurrentLimit(0);
    liftController.configContinuousCurrentLimit(35);
    liftController.configPeakCurrentDuration(100);
    liftController.enableCurrentLimit(true);

    intakeController.configFactoryDefault();
  }

  /** Configures the Shuffleboard dashboard "Hopper" tab */
  private void configureShuffleboard() {
    // Create references to Hopper tab and PID layout
    ShuffleboardTab shuffleHopperTab = Shuffleboard.getTab("Hopper");
    ShuffleboardLayout shuffleLiftPIDLayout = shuffleHopperTab
        .getLayout("Lift PID", BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "LEFT"))
        .withPosition(2, 0)
        .withSize(1, 2);
    ShuffleboardLayout shuffleLiftSetpointsLayout = shuffleHopperTab
        .getLayout("Lift Setpoints", BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "TOP"))
        .withPosition(3, 0)
        .withSize(1, 2);
    ShuffleboardLayout shuffleIntakeLayout = shuffleHopperTab
        .getLayout("Intake Modes", BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "TOP"))
        .withPosition(4, 0)
        .withSize(1, 2);

    // Configure safety override button
    subsystemEnabled = shuffleHopperTab
        .add("Subsystem Enabled", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(0, 0)
        .withSize(2,2)
        .getEntry();

    // Configure Intake and Lift setpoint indicator and controller
    liftPosition = shuffleHopperTab
        .add("Lift Position", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("Min", 0.0, "Max", 890.0))
        .withSize(4, 1)
        .withPosition(0, 2)
        .getEntry();
    liftSetpoint = shuffleHopperTab
        .add("Lift Setpoint", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("Min", 0.0, "Max", 890.0))
        .withSize(4, 1)
        .withPosition(0, 3)
        .getEntry();
    intakeSetpoint = shuffleHopperTab
        .add("Intake Setpoint", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("Min", -1.0, "Max", 1.0, "Num tick marks", 1))
        .withSize(1,1)
        .withPosition(4,2)
        .getEntry();

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

    // Configure Lift setpoints list and set default values from
    // frc.robot.Constants
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

  /** Returns whether the subsystem is enabled or not */
  public boolean getIsEnabled() {
    return subsystemEnabled.getBoolean(false);
  }

  /**
   * Checks the current status of all Hopper motors and determines if they
   * are operating within safe parameters
   */
  public boolean getIsOperatingSafely() {
    // Run several checks on the motors to ensure they are running safely
    return (liftController.getStatorCurrent() < 30.0);
  }

  /** Gets the current Lift position in sensor ticks from the Lift controller */
  public double getLiftPositionTicks() {
    return liftController.getSelectedSensorPosition();
  }

  /**
   * Gets the current Lift position in meters from the Lift controller
   * <p>WARNING: Not yet implemented
   */
  public double getLiftPositionMeters() {
    // TODO: Implement issue #18
    System.out.println("Error in frc.robot.subsystems.Hopper.getLiftPositionMeters(): Feature not yet implemented");
    return -1234.5678;
  }

  /** Gets the current Lift error in sensor ticks from the Lift controller */
  public double getLiftPositionError() {
    return liftController.getClosedLoopError();
  }

  /**
   * Gets the current Lift setpoint in sensor ticks from the NetworkTable.
   * Returns 0.0 if there is an error fetching the value.
   */
  public double getLiftSetpoint() {
    return liftSetpoint.getDouble(0.0);
  }

  /**
   * Gets the value of the specified Lift setpoint in sensor ticks from the
   * NetworkTable
   * */
  public double getLiftSetpointValue(Lift setpoint) {
    switch (setpoint) {
        case UP: {
            return liftUpSetpoint.getDouble(Subsystem.Hopper.LIFT_UP);
        }
        case DISPENSE: {
            return liftDispenseSetpoint.getDouble(Subsystem.Hopper.LIFT_DISPENSE);
        }
        case DOWN: {
            return liftDownSetpoint.getDouble(Subsystem.Hopper.LIFT_DOWN);
        }
        default: {
            throw new IllegalArgumentException("Error in frc.robot.subsystems.Hopper.getLiftSetpointValue(Lift): Cannot handle " + setpoint);
        }
    }
  }

  /**
   * Gets the current Intake setpoint in percent output (-1.0 to 1.0)
   * from the NetworkTable. Returns 0.0 if there is an error.
   */
  public double getIntakeSetpoint() {
    return intakeSetpoint.getDouble(0.0);
  }

  /**
   * Gets the value of the specified Intake setpoint in terms of percent output
   * (-1.0 to 1.0) from the NetworkTable
   */
  public double getIntakeSetpointValue(Intake setpoint) {
    switch (setpoint) {
        case IN: {
            return intakeInSpeed.getDouble(Subsystem.Hopper.INTAKE_IN);
        }
        case OUT: {
            return intakeOutSpeed.getDouble(Subsystem.Hopper.INTAKE_OUT);
        }
        case HOLD: {
            return intakeHoldCurrent.getDouble(Subsystem.Hopper.INTAKE_HOLD);
        }
        case NEUTRAL: {
            return 0.0;
        }
        default: {
            throw new IllegalArgumentException("Error in frc.robot.subsystems.Hopper.getIntakeSetpointValue(Intake): Cannot handle " + setpoint);
        }
    }
  }

  /**
   * Resets the Lift sensor position to 0
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
   * Resets the Lift setpoint value in NetworkTables to the default in
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

  /** Resets the Lift setpoint value in NetworkTables to the specified value */
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

  /** Sets the Lift setpoint to the specified operating position */
  public void setLift(Lift setpoint) {
    // Use Lift enum to set Lift output and mode
    switch (setpoint) {
        case UP: {
            setLift(
                getLiftSetpointValue(Lift.UP));
            return;
        }
        case DISPENSE: {
            setLift(
                getLiftSetpointValue(Lift.DISPENSE));
            return;
        }
        case DOWN: {
            setLift(
                getLiftSetpointValue(Lift.DOWN));
            return;
        }
        default: {
            throw new IllegalArgumentException("Error in frc.robot.subsystems.Hopper.setLiftSetpoint(Lift): Cannot handle " + setpoint);
        }
    }
  }

  /**
   * Sets the Lift setpoint to a specified position in ticks, and applies the
   * new setpoint to the controller.
   */
  public void setLift(double setpoint) {
    // Update the NetworkTables entry and set the new setpoint on the controller
    liftSetpoint.setDouble(setpoint);
  }

  /**
   * Sets the Lift setpoint to the specified height in meters
   * 
   * <p> WARNING: Not yet implemented
   */
  public void setLiftMeters(double setpoint) {
    // TODO: Implement set lift height in meters, issue #18
    System.out.println("Error in Hopper.setLiftSetpointMeters(double): Not yet implemented");
  }

  /** Sets the Intake to the specified operating mode */
  public void setIntake(Intake mode) {
    // Use Intake enum to set Intake output and mode
    switch (mode) {
        case IN: {
            setIntake(
                getIntakeSetpointValue(Intake.IN));
            return;
        }
        case OUT: {
            setIntake(
                getIntakeSetpointValue(Intake.OUT));
            break;
        }
        case HOLD: {
            // TODO: Implement current control, issue #31
            // intakeController.set(
            //     ControlMode.Current,
            //     intakeHoldCurrent.getDouble(Subsystem.Hopper.INTAKE_HOLD));
            throw new IllegalArgumentException("Error in frc.robot.subsystems.Hopper.setIntake(Intake): " + mode + " not yet implemented");
        }
        case NEUTRAL: {
            setIntake(
                getIntakeSetpointValue(Intake.NEUTRAL));
            return;
        }
        default: {
            throw new IllegalArgumentException("Error in frc.robot.subsystems.Hopper.setIntake(Intake): Cannot handle " + mode);
        }
    }
  }

  /** Sets the Intake setpoint to the specified percent output (-1.0 to 1.0) */
  public void setIntake(double percentOutput) {
    intakeSetpoint.setDouble(percentOutput);
  }

  /**
   * Sets the Intake setpoint to the specified current output between 0.0
   * and 40.0
   * 
   * @deprecated Not yet implemented, see issue #18
   */
  public void setIntakeCurrent(double currentOutput) {
    throw new IllegalCallerException("Error in frc.robot.subsystems.Hopper.setIntakeCurrent(double): Not yet implemented");
  }

  /**
   * Sets all Hopper motors to neutral, useful for safety reasons
   */
  public void setAllNeutral() {
    liftController.set(ControlMode.Disabled, 0);
    intakeController.set(ControlMode.Disabled, 0);
  }

  /**
   * Disable or enable the Hopper subsystem, and set the motors to neutral
   */
  public void setEnabled(boolean enabled) {
      // Disable or enable the subsystem
      setAllNeutral();
      subsystemEnabled.setBoolean(enabled);
  }

  /** 
   * Synchronizes member variables and motor controller values with those
   * in NetworkTables
   */
  private void syncNetworkTables() {
    // Push the Lift PIDF constants from NetworkTable to the Lift controller
    liftController.config_kP(0, liftP.getDouble(Subsystem.Hopper.LIFT_P));
    liftController.config_kI(0, liftI.getDouble(Subsystem.Hopper.LIFT_I));
    liftController.config_kD(0, liftD.getDouble(Subsystem.Hopper.LIFT_D));
    liftController.config_kF(0, liftF.getDouble(Subsystem.Hopper.LIFT_F));

    // Pull the Lift and Intake setpoints from NetworkTable to the Lift and
    // Intake controllers. If the robot is in the disabled state, switch to
    // manual control with percent output
    if (getIsEnabled()) {
        liftController.set(ControlMode.Position, getLiftSetpoint());
        intakeController.set(ControlMode.PercentOutput, getIntakeSetpoint());
    }

    // Push the current position from the Lift sensor to the NetworkTable
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
  public void configureLiftPIDF(double P, double I, double D, double F) {
    // Manually update the PIDF values in NetworkTables
    liftP.setDouble(P);
    liftI.setDouble(I);
    liftD.setDouble(D);
    liftF.setDouble(F);
  }

  @Override
  public void periodic() {
    // Synchronize motor controllers with NetworkTable values, and run a safety
    // check on the subsystem
    syncNetworkTables();
    // Perform a safety check
    // if (getIsEnabled()) {setEnabled(getIsOperatingSafely());}
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    syncNetworkTables();
    if (getIsEnabled()) {setEnabled(getIsOperatingSafely());}
  }
}
