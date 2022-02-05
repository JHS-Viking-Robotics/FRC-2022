// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /** Math and conversion constants to 6 significant figures */
  public final static class Math {
    /** The mathematical constant pi */
    public final static double PI = 3.14159;
    /** Multiply to get from inches to meters */
    public final static double INCHES_2_METERS = 0.0254000;
    /** Multiply to get from meters to inches */
    public final static double METERS_2_INCHES = 39.3701;
  }

  /** Configuration options, PID constants, and CAN Bus ID's for various subsystems */
  public final static class Subsystem {
    /** Configuration options, PID constants, and CAN Bus ID's for {@link frc.robot.subsystems.Drivetrain Drivetrain} subsystem */
    public final static class Drivetrain {
      /** CAN Bus ID of left front drive SparkMAX */
      public final static int LEFT_FRONT_ID = 3;
      /** CAN Bus ID of left back drive SparkMAX */
      public final static int LEFT_BACK_ID = 53;
      /** CAN Bus ID of right front drive SparkMAX */
      public final static int RIGHT_FRONT_ID = 1;
      /** CAN Bus ID of right back drive SparkMAX */
      public final static int RIGHT_BACK_ID = 51;
      /** Left front SparkMAX is inverted */
      public final static boolean LEFT_FRONT_INVERTED = false;
      /** Left back SparkMAX is inverted */
      public final static boolean LEFT_BACK_INVERTED = true;
      /** Right front SparkMAX is inverted */
      public final static boolean RIGHT_FRONT_INVERTED = false;
      /** Right back SparkMAX is inverted */
      public final static boolean RIGHT_BACK_INVERTED = true;
      /** P constant for SparkMAX onboard PID control */
      public final static double P = 0.0;
      /** I constant for SparkMAX onboard PID control */
      public final static double I = 0.0;
      /** D constant for SparkMAX onboard PID control */
      public final static double D = 0.0;
      /** F constant for SparkMAX onboard PID control */
      public final static double F = 0.0;
      /** Maximum forward driving velocity in m/s (meters per second). Should
       * be slightly lower than robot's maximum free speed */
      public final static double MAX_VELOCITY = 2.0;
      /** Maximum forward driving acceleration in m/s^2 (meters per second
       * squared) */
      public final static double MAX_ACCELERATION = 2.0;
      /** Maximum rotational velocity in rad/s */
      public final static double MAX_ROTATION = Math.PI/4.0;
      /** kS constant with units V (volts) for Drivetrain motor
       * characterization. See
       * <a href=https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html">FRC Docs</a>
       * section on motor characterization for more */
      public final static double S = 1.48;
      /** kV constant with units (Vs)/m (volt seconds per meter) for Drivetrain
       * motor characterization. See
       * <a href=https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html">FRC Docs</a>
       * section on motor characterization for more */
      public final static double V = 2.93;
      /** kA constant with units (Vs^2)/m (volt seconds squared per meter) for
       * Drivetrain motor characterization. See
       * <a href=https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html">FRC Docs</a>
       * section on motor characterization for more */
      public final static double A = 0.0313;
      /** MecanumDriveKinematics object for the Drivetrain */
      public final static MecanumDriveKinematics KINEMATICS = new MecanumDriveKinematics(
          new Translation2d(0.381, 0.381),
          new Translation2d(0.381, -0.381),
          new Translation2d(-0.381, 0.381),
          new Translation2d(-0.381, -0.381));
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