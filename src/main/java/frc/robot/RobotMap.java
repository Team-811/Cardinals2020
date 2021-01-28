/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

public class RobotMap {

  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  // CAN IDs

  // Drivetrain
  public static final int DRIVE_TOP_LEFT_MOTOR = 1;
  public static final int DRIVE_BOTTOM_LEFT_MOTOR = 2;
  public static final int DRIVE_TOP_RIGHT_MOTOR = 3;
  public static final int DRIVE_BOTTOM_RIGHT_MOTOR = 4;

  public static final int SHOOTER = 5;

  public static final int KICKER = 6;

  public static final int INTAKE_STORAGE = 7;
  public static final int WHEEL = 8;
  public static final int INTAKE_SPINNER = 14;

  // Extra SparkMax ID: 9

  public static final int TELESCOPE_WINCH = 10;  

  public static final int TELESCOPE_EXTEND = 11;
  public static final int WINCH = 12;
  

  // Sensors
  //public static final int TELESCOPE_LOW = 9;
  public static final int TELESCOPE_HIGH = 8;
  public static final int TELESCOPE_UP = 0;
  
}
