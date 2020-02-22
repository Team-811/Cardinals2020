/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * This class contains constants about the robot like physical dimensions
 */
public class Constants {
    // Speed constants

    public static final double DRIVETRAIN_SPEED_SCALE = 0.7;
    public static final double DRIVETRAIN_SLOW_SCALE = 0.3;

    public static final double INTAKE_STORAGE_SPEED = 0.6;

    public static final double SHOOTER_SPEED = 1;
    public static final double KICKER_SPEED = 1;

    public static final double SHOOTER_FULL_VELOCITY = 5100;

    public static final double DRIVETRAIN_RAMP_RATE = 0.75;

    // Conversion Factors for Encoders

    /**
     * 24.34 inches = 1 ticks 1 inch = 0.041 ticks
     */
    public static final double TICKS_TO_INCHES_WHEELS = 7.75 * Math.PI * .1;

    /**
     * 1 rotation = 10 ticks
     */
    public static final double TICKS_TO_ROTATIONS_WHEELS = 0.1;

    // Field Dimensions (values in inches)
    public static final double LINE_TO_WALL = 120;

    // Robot Dimensions (values in inches)

    // With bumpers
    public static final double ROBOT_LENGTH = 45;
    public static final double ROBOT_WIDTH = 0;

}
