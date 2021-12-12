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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

  private final WPI_TalonSRX liftController;
  private final WPI_VictorSPX intakeController;

  private int safetyScore;           // Current safety score of subsystem [0, 100]
  private boolean subsystemEnabled;  // Is the subsystem currently enabled

  private NetworkTableEntry liftP;
  private NetworkTableEntry liftI;
  private NetworkTableEntry liftD;
  private NetworkTableEntry liftF;
  private NetworkTableEntry liftSetpoint;
  private NetworkTableEntry liftPosition;
  private NetworkTableEntry intakeSetpoint;

  /** Hopper Intake modes */
  public enum Intake {
    IN(Subsystem.Hopper.INTAKE_IN),
    OUT(Subsystem.Hopper.INTAKE_OUT),
    HOLD(Subsystem.Hopper.INTAKE_HOLD),
    NEUTRAL(0.0);

    private double speed;

    private Intake(double speed) {this.speed = speed;}

    public double getValue() {return this.speed;}
  }

  /** Hopper Lift positions */
  public enum Lift {
    UP(Subsystem.Hopper.LIFT_UP),
    DISPENSE(Subsystem.Hopper.LIFT_DISPENSE),
    DOWN(Subsystem.Hopper.LIFT_DOWN),
    NEUTRAL(0.0);

    private double position;
    
    private Lift(double position) {this.position = position;}
    
    public double getValue() {return this.position;}
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

    // Start the safety score at maximum, and enable the subsystem
    safetyScore = 100;
    subsystemEnabled = true;
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
    liftController.configContinuousCurrentLimit(30);
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
        .withProperties(
            Map.of(
                "Label position", "LEFT"))
        .withPosition(2, 0)
        .withSize(1, 2);

    // Configure Intake and Lift setpoint indicator and controller
    liftPosition = shuffleHopperTab
        .add("Lift Position", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            Map.of(
                "Min", Subsystem.Hopper.LIFT_DOWN,
                "Max", Subsystem.Hopper.LIFT_UP))
        .withSize(4, 1)
        .withPosition(0, 2)
        .getEntry();
    liftSetpoint = shuffleHopperTab
        .add("Lift Setpoint", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(
            Map.of(
                "Min", Subsystem.Hopper.LIFT_DOWN,
                "Max", Subsystem.Hopper.LIFT_UP))
        .withSize(4, 1)
        .withPosition(0, 3)
        .getEntry();
    intakeSetpoint = shuffleHopperTab
        .add("Intake Setpoint", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            Map.of(
                "Min", -1.0,
                "Max", 1.0,
                "Num tick marks", 1))
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
  }

  /** Returns whether the subsystem is enabled or not */
  public boolean isEnabled() {
    return subsystemEnabled;
  }

  /**
   * Change the safety score for the subsystem by some amount
   * 
   * @param change amount +/- to change safety score by
   */
  private void changeSafetyScore(int change) {
    safetyScore += change;
    if (safetyScore < 0) {safetyScore = 0;}
    if (safetyScore > 100) {safetyScore = 100;}
  }

  /**
   * Do some checks on the subsystem and update the safety rating accordingly.
   * 
   * Will slowly return the rating to maximum if everything is working correctly.
   */
  private void performSafetyTest() {
    // Ensure the safety score is able to return to 100 by slowly increasing it
    changeSafetyScore(1);

    // Calculate the maximum range of motion for the Lift, plus some wiggle room
    double maxRangeMotion = 200 + Math.abs(
        Lift.UP.getValue() - Lift.DOWN.getValue());

    // Run checks on the motors to ensure they are running safely. Note that
    // the score should be decreased by 1 extra unit to compensate for this
    // method trying to return the score to maximum
    if (liftController.getStatorCurrent() > 10.0) {
      changeSafetyScore(-2);
    } if (getLiftPositionError() > maxRangeMotion) {
      changeSafetyScore(-21);
    }

    // Disable the subsystem if we are not operating safely, otherwise check
    // the dashboard button to see if we are disabled there
    if (safetyScore < 10) {
      setSubsystemEnabled(false);
    } else {
      setSubsystemEnabled(isEnabled());
    }
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
   * Gets the current Intake setpoint in percent output (-1.0 to 1.0)
   * from the NetworkTable
   */
  public double getIntakeSetpoint() {
    return intakeSetpoint.getDouble(0.0);
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

  /** Sets the Lift setpoint to the specified operating position
   * 
   * @param setpoint desired Lift position
   * 
   * @see #setLift(double)
   */
  public void setLift(Lift setpoint) {
    setLift(setpoint.getValue());
  }

  /**
   * Sets the Lift setpoint to a specified position in ticks, and applies the
   * new setpoint to the controller.
   * 
   * @param setpoint desired encoder position to send to the controller
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
    setIntake(mode.getValue());
    // TODO: Implement current control, issue #31
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
   * Sets all Hopper motors to neutral
   */
  public void setAllNeutral() {
    liftController.set(ControlMode.Disabled, 0);
    intakeController.set(ControlMode.Disabled, 0);
  }

  /**
   * Disable or enable the Hopper subsystem, and set the motors to neutral
   */
  public void setSubsystemEnabled(boolean enabled) {
    // We are already in desired state, so skip everything
    if (subsystemEnabled == enabled) {
      return;
    }
  
    // Update subsystemEnabled and get a reference to the CommandScheduler
    CommandScheduler cmd = CommandScheduler.getInstance();
    subsystemEnabled = enabled;
    if (enabled) {
      // Re-enable the subsystem and clear any running commands
      subsystemEnabled = enabled;
      cmd.cancel(cmd.requiring(this));
    } else {
      // Disable the subsystem by cancelling all commands and reverting back
      // to neutral
      subsystemEnabled = enabled;
      setAllNeutral();
      cmd.cancel(cmd.requiring(this));
    }
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
    if (isEnabled()) {
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
    performSafetyTest();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    syncNetworkTables();
    performSafetyTest();
  }
}
