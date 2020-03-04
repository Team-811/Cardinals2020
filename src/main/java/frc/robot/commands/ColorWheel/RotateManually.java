/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ColorWheel;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class RotateManually extends Command {

  /**
   * This command manually spins the color wheel
   */
  public RotateManually() {
  }

  // spin the wheel while a button is held down
  @Override
  protected void execute() {
    Robot.colorWheel.rotationControlManual(Constants.COLOR_WHEEL_SPEED);
  }

  // stop the color wheel when the command is no longer being called
  @Override
  protected void end() {
    Robot.colorWheel.stopColorWheel();
    Robot.setDefaultLED();
  }

  @Override
  protected void interrupted() {
    end();
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
