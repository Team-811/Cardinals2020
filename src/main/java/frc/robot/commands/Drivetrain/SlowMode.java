/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SlowMode extends Command {
  public SlowMode() {
    
  }
  
  @Override
  protected void execute() {
    Robot.drivetrain.slowMode(true);
  }

  @Override
  protected void end(){
    Robot.drivetrain.slowMode(false);
  }

  @Override
  protected void interrupted() {
    Robot.drivetrain.slowMode(false);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
