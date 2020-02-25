/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ColorFill extends InstantCommand {

    /**
     * Sets LEDs to one color filling up the strand at given delay
     */
    public ColorFill(int HSVcolor, int delay) {
        _delay = delay;
        _HSVcolor = HSVcolor;
    }

    private int _delay;
    private int _HSVcolor;

    // Called once when the command executes. Since this is an InstantCommand, the
    // method will only be called once and not continuously. This is good when
    // something needs to be turned on/off or toggled.
    @Override
    protected void initialize() {
        Robot.led.setColorFill(_HSVcolor, _delay);
    }

}
