// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /** Math and conversion constants to 6 significant figures.
   * 
   * <p>NOTE: The library {@link edu.wpi.first.wpiutil.math.MathUtil WPI MathUtil Class} contains non-static
   * methods for several of these conversions, but these static constants are
   * available for the constants chassis measurements in
   * frc.robot.Constants.Chassis
  */
  public final class Math {
    /** The mathematical constant pi */
    public final static double PI = 3.14159;
    /** Multiply to get from inches to meters */
    public final static double INCHES_2_METERS = 0.0254000;
    /** Multiply to get from meters to inches */
    public final static double METERS_2_INCHES = 39.3701;

  }
  /** Configuration options, PID constants, and CAN Bus ID's for various subsystems */
  public final class Subsystem {

    /** Configuration options, PID constants, and CAN Bus ID's for {@link frc.robot.subsystems.Drivetrain Drivetrain} subsystem */
    public final class Drivetrain {
      /** CAN Bus ID of primary left drive Talon */
      public final static int LEFT_MAIN = 3;
      /** CAN Bus ID of follower left drive Talon or Victor */
      public final static int LEFT_FOLLOW = 53;
      /** CAN Bus ID of primary right drive Talon */
      public final static int RIGHT_MAIN = 1;
      /** CAN Bus ID of follower right drive Talon or Victor */
      public final static int RIGHT_FOLLOW = 51;
      /** Left side Talons are inverted */
      public final static boolean LEFT_INVERTED = false;
      /** Right side Talons are inverted */
      public final static boolean RIGHT_INVERTED = true;
      /** P constant for Talon onboard PID control */
      public final static double P = 10.0;
      /** I constant for Talon onboard PID control */
      public final static double I = 0.0;
      /** D constant for Talon onboard PID control */
      public final static double D = 0.0;
      /** F constant for Talon onboard PID control */
      public final static double F = 0.0;
      /** Maximum forward driving velocity in m/s */
      public final static double MAX_VELOCITY = 0.25;
      /** Maximum rotational velocity in rad/s */
      public final static double MAX_ROTATION = Math.PI/4.0;
    }

    /** Configuration options, PID constants, and CAN Bus ID's for {@link frc.robot.subsystems.Hopper Hopper} subsystem */
    public final class Hopper {
      /** CAN Bus ID of lift Talon controller */
      public final static int LIFT_ID = 2;
      /** CAN Bus ID of intake Victor controller */
      public final static int INTAKE_ID = 54;
      /** Lift Talon controller is inverted */
      public final static boolean LIFT_CONTROLLER_INVERTED = true;
      /** Lift Talon sensor is inverted */
      public final static boolean LIFT_SENSOR_INVERTED = false;
      /** Intake Victor is inverted */
      public final static boolean INTAKE_CONTROLLER_INVERTED = true;
      /** P constant for Lift Talon onboard PID control */
      public final static double LIFT_P = 1.0;
      /** I constant for Lift Talon onboard PID control */
      public final static double LIFT_I = 0.0;
      /** D constant for Lift Talon onboard PID control */
      public final static double LIFT_D = 0.0;
      /** F constant for Lift Talon onboard PID control */
      public final static double LIFT_F = 0.0;
      /** Tick position for Lift Up setpoint */
      public final static double LIFT_UP = 890.0;
      /** Tick position for Lift Dispense setpoint */
      public final static double LIFT_DISPENSE = 728.0;
      /** Tick position for Lift Down setpoint */
      public final static double LIFT_DOWN = 0.0;
      /** Percent output for Intake In mode */
      public final static double INTAKE_IN = 0.5;
      /** Percent output for Intake Out mode */
      public final static double INTAKE_OUT = 0.5;
      /** Current output for Intake Hold mode */
      public final static double INTAKE_HOLD = 0.5;
    }
  }

  /** Joystick USB ports */
  public final class Joystick {
    /** USB port for driver controller */
    public final static int DRIVER = 0;
    /** USB port for support controller */
    public final static int SUPPORT = 1;
  }

  /** Physical chassis measurements and dimensions */
  public final class Chassis {
    /** Wheel diameter in meters */
    public final static double WHEEL_DIAMETER = 6 * Math.INCHES_2_METERS;
    /** Wheel circumference in meters */
    public final static double WHEEL_CIRCUM = WHEEL_DIAMETER * Math.PI;
    /** Width between two wheels (axle length) in meters */
    public final static double TRACK_WIDTH = 23 * Math.INCHES_2_METERS;
    /** Width of the robot in meters */
    public final static double WIDTH = 27 * Math.INCHES_2_METERS;
    /** Length of the robot in meters */
    public final static double LENGTH = 32.3 * Math.INCHES_2_METERS;
  }
}
