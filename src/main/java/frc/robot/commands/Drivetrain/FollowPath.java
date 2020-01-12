/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import jaci.pathfinder.Waypoint;

/**
 * This is a command.  A command is used to make actual actions happen on the robot.  It can be a single action or a sequence of actions.
 */

public class FollowPath extends Command {

  private Waypoint[] path;
  private boolean reverse;

  public FollowPath(Waypoint[] path, boolean reverse) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.drivetrain);
    this.path = path;
    this.reverse = reverse;

  }

  public FollowPath(Waypoint[] path) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.drivetrain);
    this.path = path;
    this.reverse = false;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivetrain.loadTrajectory(path, reverse);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drivetrain.followTrajectory(reverse);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.drivetrain.isPathFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.stopDrivetrain();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
