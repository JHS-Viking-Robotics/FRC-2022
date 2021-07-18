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
   * NOTE: The library epu.wpi.first.wpiutil.math.MathUtil contains non-static
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
  /** Talon CAN Bus ID's for various subsystems */
  public final class Talon {

    /** Talon CAN Bus ID for {@link Drivetrain} subsystem */
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
      public final static boolean RIGHT_INVERTED = false;
    }

  }

  /** Joystick USB ports */
  public final class Joystick {
    /** USB port for driver controller */
    public final static int DRIVER = 0;
    /** USB port for support controller */
    public final static int SUPPORT = 1;
  }

  /** Chassis measurements */
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
