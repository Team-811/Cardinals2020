/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Utility;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class TimerCommand extends TimedCommand {
  /**
   * This command pauses the code for a given period between commands in command
   * groups.
   * 
   * @param timeout : time in seconds
   */
  public TimerCommand(double timeout) {
    super(timeout);
  }

  /**
   * Used to wait in the code (not between commands)
   * 
   * @param seconds
   */
  public static void PauseCode(double seconds) {
    double start = Timer.getFPGATimestamp();
    while (Timer.getFPGATimestamp() - start < seconds) {
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Called once after timeout
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
