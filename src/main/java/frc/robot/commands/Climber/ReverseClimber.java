/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ReverseClimber extends Command {
  /**
   * This command puts the climber in reverse while active
   */
  public ReverseClimber() {
  }

  // Called continously while the command is run. In this case, while the button
  // for reverse mode is pressed.
  @Override
  protected void execute() {
    Robot.climber.reverse(true);
  }

  // turn off reverse mode when the command is not longer being called
  @Override
  protected void end() {
    Robot.climber.reverse(false);
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
