/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.controllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Auto.CrossLine;
import frc.robot.commands.Auto.CrossLineAndShootComp;
import frc.robot.commands.Climber.ReverseClimber;
import frc.robot.commands.Climber.RunTelescopeWinch;
import frc.robot.commands.ColorWheel.RotateManually;
import frc.robot.commands.ColorWheel.RotationControlTimed;
import frc.robot.commands.Drivetrain.SlowMode;
import frc.robot.commands.Drivetrain.ToggleDriveMode;
import frc.robot.commands.IntakeStorage.RunIntakeStorageReverse;
import frc.robot.commands.IntakeStorage.ToggleIntakeStorage;
import frc.robot.commands.Shooter.AutoRunShooter;
import frc.robot.commands.Shooter.ToggleKickerAndShooter;
import frc.robot.commands.Utility.ZeroSensors;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  /**
   * Switch between single or dual controller modes for competition/testing. True
   * is dual controller and false is single.
   */
  private boolean mode = false;

  public BobXboxController driveController;
  public BobXboxController operatorController;

  public static OI getInstance() {
    return new OI();
  }

  /**
   * Operator interface for binding commands to controllers
   */
  public OI() {
    driveController = new BobXboxController(0, 0.3, 0.3);
    operatorController = new BobXboxController(1, 0.3, 0.3);

    // Dual Controller
    if (mode) {

      // Driver
      driveController.startButton.whenPressed(new ToggleDriveMode());
      driveController.selectButton.whileHeld(new ReverseClimber());
      driveController.leftBumper.whileHeld(new SlowMode());
      driveController.xButton.whileHeld(new RunTelescopeWinch());
      driveController.aButton.whileHeld(new ZeroSensors());      
      //driveController.yButton.whileHeld(new RunTelescopeExtend());
      //driveController.bButton.whileHeld(new RunWinch());

      // Operator
      operatorController.leftBumper.whileHeld(new RotateManually());
      operatorController.rightBumper.whenPressed(new RotationControlTimed(4));

      operatorController.rightTriggerButton.whileHeld(new AutoRunShooter());
      operatorController.leftTriggerButton.whileHeld(new RunIntakeStorageReverse());
      operatorController.aButton.whenPressed(new ToggleIntakeStorage());
    
      operatorController.yButton.whenPressed(new ToggleKickerAndShooter());
    }

    // Single controller
    else {
      driveController.startButton.whenPressed(new ToggleDriveMode());
      driveController.selectButton.whenPressed(new ZeroSensors());

      driveController.leftBumper.whileHeld(new SlowMode());

      driveController.leftTriggerButton.whileHeld(new RunIntakeStorageReverse());
      driveController.rightTriggerButton.whileHeld(new AutoRunShooter());

      driveController.aButton.whenPressed(new ToggleIntakeStorage());
      
      driveController.bButton.whenPressed(new ToggleKickerAndShooter());

      driveController.xButton.whileHeld(new RunTelescopeWinch());
      driveController.yButton.whileHeld(new ReverseClimber());

      driveController.rightBumper.whileHeld(new RotateManually());

      //driveController.yButton.whenPressed(new CrossLine());
      //driveController.rightBumper.whenPressed(new CrossLineAndShootComp());

    }

  }

  public void outputSmartDashboard() {
    String strMode;
    if (mode) {
      strMode = "DUAL";
    } else
      strMode = "SINGLE";

    SmartDashboard.putString("Controller Mode", strMode);
  }

}
