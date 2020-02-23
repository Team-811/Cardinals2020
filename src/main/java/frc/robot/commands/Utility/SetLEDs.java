/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Utility;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class SetLEDs extends InstantCommand {

  /**
   * This command sets the LEDs on the robot to a given preset.
   */
  public SetLEDs(int pattern) {
    _pattern = pattern;
  }

  private int _pattern;

  /* LED Presets:
    0 "ERROR"
    1 "Rainbow Dot Trail"
    2 "Rainbow"
    3 "Rainbow Gradient"
    4 "Rainbow Snake",
    5 "Red/Yellow Snake"
    6 "Blue/Yellow Snake"
    7 "Confetti"
    8 "America"
    9 "Red Flame"
   10 "Blue Flame"
   11 "BG Flame",
   12 "Rainbow Flame"
   13 "Rainbow Chase"
   14 "Red"
   15 "Green"      
  */

  // Called once when the command executes. Since this is an InstantCommand, the
  // method will only be called once and not continuously. This is good when
  // something needs to be turned on/off or toggled.
  @Override
  protected void initialize() {
    Robot.led.setLEDs(_pattern);
  }

}
