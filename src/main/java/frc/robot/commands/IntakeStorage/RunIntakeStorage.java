/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.IntakeStorage;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class RunIntakeStorage extends Command {

  /**
   * This command runs the intake at a given speed while active
   */
  public RunIntakeStorage() {
  }

  // run the intake/storage continously while the command is called
  @Override
  protected void execute() {
    Robot.intakeStorage.runIntakeStorage(Constants.INTAKE_STORAGE_SPEED);
  }

  // stop the intake/storage when the command is no longer being called
  @Override
  protected void end() {
    Robot.intakeStorage.stopIntakeStorage();
    Robot.intakeStorage.setIntakeLEDs(false);
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
