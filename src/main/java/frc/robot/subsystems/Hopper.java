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

  private final WPI_TalonSRX liftController;    // Talon controller for Lift motor
  private final WPI_VictorSPX intakeController; // Victor controller for Intake motor

  private int safetyScore;          // Current safety score of subsystem [0, 100]
  private boolean subsystemEnabled; // Is the subsystem currently enabled

  private NetworkTableEntry liftP; // kP for Lift Talon closed-loop PID
  private NetworkTableEntry liftI; // kI for Lift Talon closed-loop PID
  private NetworkTableEntry liftD; // kD for Lift Talon closed-loop PID
  private NetworkTableEntry liftF; // kF for Lift Talon closed-loop PID
  private NetworkTableEntry liftSetpoint;   // In test mode, desired Lift setpoint in Talon sensor ticks
  private NetworkTableEntry liftPosition;   // NetTables dashboard indicator for Lift position in Talon sensor ticks
  private NetworkTableEntry intakeSetpoint; // In test mode, desired Intake setpoint in PercentOutput [-1,1]

  /** Hopper Intake modes of operation */
  public enum Intake {
    /** Hopper Intake pulling in mode */
    IN(ControlMode.PercentOutput, Subsystem.Hopper.INTAKE_IN),
    /** Hopper Intake pushing out mode */
    OUT(ControlMode.PercentOutput, Subsystem.Hopper.INTAKE_OUT),
    /** Hopper Intake hold mode */
    HOLD(ControlMode.Current, Subsystem.Hopper.INTAKE_HOLD),
    /** Hopper Intake motor output disabled */
    NEUTRAL(ControlMode.Disabled, 0.0),
    /** Hopper Intake manual control mode using NetworkTables input.
     * <p> Used for testing the Intake */
    TESTING(ControlMode.PercentOutput, 0.0);

    private double output;
    private ControlMode mode;

    private Intake(ControlMode mode, double output) {
      this.output = output;
      this.mode = mode;
    }

    /** Get the motor output value */
    public double getValue() {return this.output;}
    /** Get the motor control mode */
    public ControlMode getMode() {return this.mode;}
  }

  /** Hopper Lift modes of operation */
  public enum Lift {
    /** Hopper Lift p position */
    UP(ControlMode.Position, Subsystem.Hopper.LIFT_UP),
    /** Hopper Lift dispense position */
    DISPENSE(ControlMode.Position, Subsystem.Hopper.LIFT_DISPENSE),
    /** Hopper Lift down position */
    DOWN(ControlMode.Position, Subsystem.Hopper.LIFT_DOWN),
    /** Hopper Lift neutral mode, motor output disabled */
    NEUTRAL(ControlMode.Disabled, 0.0),
    /** Hopper Lift manual control mode using NetworkTables input.
     * <p> Used for testing the Lift */
    TESTING(ControlMode.Position, 0.0);

    private double output;
    private ControlMode mode;

    private Lift(ControlMode mode, double output) {
      this.output = output;
      this.mode = mode;
    }

    /** Get the motor output value */
    public double getValue() {return this.output;}
    /** Get the motor control mode */
    public ControlMode getMode() {return this.mode;}
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
   * Disables the subsystem if the test fails, and re-enables it if the test
   * passes 10 consecutive times.
   * 
   * <p> Note that this test should be ran in {@link #periodic()} every cycle
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
   * 
   * @deprecated
   */
  public double getLiftPositionMeters() {
    // TODO: Implement issue #18
    System.out.println("Error in frc.robot.subsystems.Hopper.getLiftPositionMeters(): Feature not yet implemented");
    throw new IllegalCallerException();
  }

  /** Gets the current Lift error in sensor ticks from the Lift controller */
  public double getLiftPositionError() {
    return liftController.getClosedLoopError();
  }

  /**
   * Gets the current Lift setpoint in sensor ticks from the NetworkTable
   */
  public double getLiftSetpoint() {
    return liftSetpoint.getDouble(0.0);
  }

  /**
   * Gets the current Intake setpoint in percent output [-1.0, 1.0]
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

  /**
   * Sets the Lift to the specified operating mode.
   * 
   * <p> Will do nothing and print a warning to the console if the subsystem
   * is disabled
   * 
   * @param mode desired Lift operating mode
   */
  public void setLift(Lift mode) {
    if (!isEnabled()) {
      System.out.println("WARNING: Hopper subsystem is disabled");
      return;
    }
    switch (mode) {
      case TESTING:
        liftController.set(ControlMode.Position, getLiftSetpoint());
        break;
      default:
        if (mode.getMode() == ControlMode.PercentOutput) {
          liftSetpoint.setDouble(mode.getValue());
        }
        liftController.set(mode.getMode(), mode.getValue());
        break;
    }
  }

  /**
   * Manually sets the Lift motor to the specified output between [-1, 1]
   * 
   * @param percentOut desired motor output as a percent of maximum
   * 
   * @see #setLift(Lift)
   */
  public void setLift(double percentOut) {
    liftController.set(ControlMode.PercentOutput, percentOut);
  }

  /**
   * Sets the Lift setpoint to the specified height in meters
   * 
   * <p> WARNING: Not yet implemented
   */
  private void setLiftMeters(double setpoint) {
    // TODO: Implement set lift height in meters, issue #18
    System.out.println("Error in Hopper.setLiftSetpointMeters(double): Not yet implemented");
    throw new IllegalAccessError();
  }

  /**
   * Sets the Intake to the specified operating mode
   * 
   * @param mode desired Intake operating mode
   */
  public void setIntake(Intake mode) {
    if (!isEnabled()) {
      System.out.println("WARNING: Hopper subsystem is disabled");
      return;
    }
    switch (mode) {
      case TESTING:
        intakeController.set(ControlMode.PercentOutput, getIntakeSetpoint());
        break;
      default:
        if (mode.getMode() == ControlMode.PercentOutput) {
          liftSetpoint.setDouble(mode.getValue());
        }
        intakeController.set(mode.getMode(), mode.getValue());
        break;
    }
  }

  /**
   * Manually sets the Intake motor to the specified output between [-1, 1]
   * 
   * @param percentOutput desired motor output as a percent of maximum
   * 
   * @see #setIntake(Intake)
   */
  public void setIntake(double percentOutput) {
    intakeController.set(ControlMode.PercentOutput, percentOutput);
  }

  /**
   * Sets all Hopper motors to neutral
   */
  public void setAllNeutral() {
    setLift(Lift.NEUTRAL);
    setIntake(Intake.NEUTRAL);
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
      // Re-enable the subsystem. Don't clear running commands, so we don't
      // interrupt Manual mode
      subsystemEnabled = enabled;
    } else {
      // Disable the subsystem by cancelling all commands and reverting back
      // to neutral
      System.out.println("WARNING: Hopper subsystem has detected unsafe conditions and is automatically disabling itself");
      subsystemEnabled = enabled;
      cmd.cancel(cmd.requiring(this));
      // Note that we call setAllNeutral after cancelling active commands. If
      // Command.end() gets called, it could update the motors again.
      setAllNeutral();
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
