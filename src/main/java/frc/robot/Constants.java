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
    /** Configuration options, PID constants, and CAN Bus ID's for {@link frc.robot.subsystems.Intake Intake} subsystem */
    public final static class Intake {
      /** CAN Bus ID of Intake SparkMAX */
      public final static int TAKE_ID = 22;
      /** Intake SparkMAX is inverted */
      public final static boolean TAKE_INVERTED = false;
      /** ID for pneumatic channel for dropping intake*/
      public final static int INTAKE_DROP_ID = 3;
      /** Max speed of the robot between [0,1] */
      public final static double TAKE_SPEED = 0.85;
    }

    /** Configuration options, PID constants, and CAN Bus ID's for {@link frc.robot.subsystems.Shooter Shooter} subsystem */
    public final static class Shooter {
      /** CAN Bus ID of front SparkMAX */
      public final static int FRONT_ID = 32;
      /** CAN Bus ID of rear SparkMAX */
      public final static int REAR_ID = 33;
      /** Forward channel on the Pneumatics Module */
      public final static int TRIGGER_FORWARD_ID = 1;
      /** Reverse channel on the Pneumatics Module */
      public final static int TRIGGER_REVERSE_ID = 2;
      /** Firing speed of the motors [0.0, 1.0] */
      public final static double SHOOTER_SPEED = 0.75;
      /** Front SparkMAX is inverted */
      public final static boolean FRONT_INVERTED = true;
      /** Rear SparkMAX is inverted */
      public final static boolean REAR_INVERTED = false;
    }

    /** Configuration options, PID constants, and CAN Bus ID's for {@link frc.robot.subsystems.Drivetrain Drivetrain} subsystem */
    public final static class Drivetrain {
      /** CAN Bus ID of left front drive SparkMAX */
      public final static int LEFT_FRONT_ID = 48;
      /** CAN Bus ID of left back drive SparkMAX */
      public final static int LEFT_BACK_ID = 50;
      /** CAN Bus ID of right front drive SparkMAX */
      public final static int RIGHT_FRONT_ID = 49;
      /** CAN Bus ID of right back drive SparkMAX */
      public final static int RIGHT_BACK_ID = 51;
      /** Left front SparkMAX is inverted */
      public final static boolean LEFT_FRONT_INVERTED = false;
      /** Left back SparkMAX is inverted */
      public final static boolean LEFT_BACK_INVERTED = false;
      /** Right front SparkMAX is inverted */
      public final static boolean RIGHT_FRONT_INVERTED = true;
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
      /** Max speed of the robot between [0,1] */
      public final static double MAX_SPEED = 0.75;
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

    /** Configuration options, PID constants, and CAN Bus ID's for {@link frc.robot.subsystems.Lift Lift} subsystem */
    public final static class Lift {
      /** CAN Bus ID of lift SparkMAX */
      public final static int LIFT_ID = 24;
      /** Lift SparkMAX is inverted */
      public final static boolean LIFT_INVERTED = true;
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