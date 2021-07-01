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

  /** Talon CAN Bus ID's for various subsystems */
  public final class Talon {

    /** Talon CAN Bus ID for {@link Drivetrain} subsystem */
    public final class Drivetrain {
      /** CAN Bus ID of primary left drive Talon */
      public final static int LEFT_MAIN = 1;
      /** CAN Bus ID of follower left drive Talon or Victor */
      public final static int LEFT_FOLLOW = 51;
      /** CAN Bus ID of primary right drive Talon */
      public final static int RIGHT_MAIN = 2;
      /** CAN Bus ID of follower right drive Talon or Victor */
      public final static int RIGHT_FOLLOW = 52;
    }

  }

  /** Joystick USB ports */
  public final class Joystick {
    /** USB port for driver controller */
    public final static int DRIVER = 0;
    /** USB port for support controller */
    public final static int SUPPORT = 1;
  }

}
