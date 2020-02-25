/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ColorChase extends InstantCommand {

    /**
     * Sets LEDs to chase pattern with two colors (set both to same for one color)
     * and given delay
     */
    public ColorChase(int HSVcolor1, int HSVcolor2, int delay) {
        _HSVcolor1 = HSVcolor1;
        _HSVcolor2 = HSVcolor2;
        _delay = delay;
    }

    private int _HSVcolor1;
    private int _HSVcolor2;
    private int _delay;

    // Called once when the command executes. Since this is an InstantCommand, the
    // method will only be called once and not continuously. This is good when
    // something needs to be turned on/off or toggled.
    @Override
    protected void initialize() {
        Robot.led.setColorChase(_HSVcolor1, _HSVcolor2, _delay);
    }

}
