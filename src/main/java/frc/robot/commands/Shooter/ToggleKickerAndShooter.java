/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;

public class ToggleKickerAndShooter extends InstantCommand {

  /**
   * This Command toggles the kicker and shooter on/off. Use this when the auto
   * shooter command won't work due to low battery
   */
  public ToggleKickerAndShooter() {

  }

  // Called once when the command executes. Since this is an InstantCommand, the
  // method wil only be called once and not continuously. This is good when
  // something needs to be turned on/off or toggled.
  @Override
  protected void initialize() {
    Robot.shooter.toggleKickerAndShooter(Constants.SHOOTER_SPEED, Constants.KICKER_SPEED);
  }

}
