/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.IntakeStorage;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RunIntakeStorageReverse extends Command {

  /**
   * This command runs the intake backward at a given speed while active
   */
  public RunIntakeStorageReverse() {
  }

  // run the intake/storage continously while the command is called
  @Override
  protected void execute() {
    Robot.intakeStorage.runIntakeStorage(-1);
  }

  // stop the intake/storage when the command is no longer being called
  @Override
  protected void end() {
    Robot.intakeStorage.stopIntakeStorage();
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
