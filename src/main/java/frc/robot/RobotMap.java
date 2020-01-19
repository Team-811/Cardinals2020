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

  //CAN IDs

  //Drivetrain - Neos
  public static final int DRIVE_TOP_LEFT_MOTOR = 1;
  public static final int DRIVE_BOTTOM_LEFT_MOTOR = 2;
  public static final int DRIVE_TOP_RIGHT_MOTOR = 3; 
  public static final int DRIVE_BOTTOM_RIGHT_MOTOR = 4;
  
  //Shooter
  public static final int SHOOTER = 5; //Neo
  public static final int SHOOTER_INTAKE = 6; //Victor

  //Intake
  public static final int INTAKE = 7; //Victor
  
  //Storage
  public static final int STORAGE = 8; //Talon
  public static final int STORAGE_INTAKE = 9; //Victor

  //Climber
  public static final int WINCH = 10; //Neo
  public static final int HOOK = 11; //Talon

  //Wheel
  public static final int WHEEL = 12; //Talon

  //Sensors


}
