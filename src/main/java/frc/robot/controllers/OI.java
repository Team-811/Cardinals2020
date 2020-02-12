/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.controllers;

import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.IntakeStorage.*;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.Utility.*;

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

  public BobXboxController driveController;
  public BobXboxController operatorController;

  public OI() {
    driveController = new BobXboxController(0, 0.3, 0.3);

    driveController.leftBumper.whileHeld(new SlowMode());
    driveController.rightBumper.whenPressed(new ZeroSensors());   
    
    driveController.startButton.whenPressed(new ToggleDriveMode());
    
    driveController.leftTriggerButton.whenPressed(new UnjamComp());
    driveController.rightTriggerButton.whileHeld(new AutoRunShooter());

    driveController.yButton.whenPressed(new ToggleIntakeStorage());

    driveController.bButton.whenPressed(new ToggleShooter());
    driveController.xButton.whenPressed(new ToggleKicker());
    
    operatorController = new BobXboxController(1, 0.3, 0.3);

  }

}
