/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class AutoRunShooter extends Command {

  /**
   * This command runs the shooter (and kicker) automatically accounting for lost
   * shooter speed because of balls
   */
  public AutoRunShooter() {
  }

  // run the shooter automatically, continously while the command is called (while
  // a button is held)
  @Override
  protected void execute() {
    Robot.shooter.autoRunShooter(Constants.SHOOTER_SPEED, Constants.KICKER_SPEED);
  }

  // stop the shooter when the command is no longer being called
  @Override
  protected void end() {
    Robot.shooter.stopShooter();
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
