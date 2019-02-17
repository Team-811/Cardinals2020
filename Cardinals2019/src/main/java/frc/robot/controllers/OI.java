/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.controllers;

import frc.robot.controllers.Utility.*;
import frc.robot.commands.Intakes.CommandGroups.*;
import frc.robot.commands.Intakes.InstantCommands.*;

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

    public OI()
    {
      driveController = new BobXboxController(0, 0.2, 0.2);
      operatorController = new BobXboxController(1, 0.2, 0.2);

      operatorController.aButton.whenPressed(new ReleaseHatch());
      operatorController.bButton.whenPressed(new GrabHatch());
      operatorController.xButton.whenPressed(new PlaceHatchComp());
      operatorController.yButton.whenPressed(new AquireHatchComp());  
      operatorController.rightBumper.whenPressed(new IntakeCargoComp());
      operatorController.leftBumper.whenPressed(new OuttakeBallComp());
      operatorController.startButton.whenPressed(new DropCargoIntake());
      operatorController.selectButton.whenPressed(new BringUpCargoIntake());

    }

  

}
