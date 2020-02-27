/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class RunWinch extends Command {

  /**
   * This command runs the winch on the climber to lift the robot
   */
  public RunWinch() {
  }

  //run the winch while a button is pressed
  @Override
  protected void execute() {
    Robot.climber.runWinch(Constants.WINCH_SPEED);
  }

  // stop the winch when the command is no longer being called
  @Override
  protected void end() {
    Robot.climber.runWinch(0);
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
