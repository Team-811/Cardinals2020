/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class SetLEDColor extends InstantCommand {
    /**
     * This command sets the color of the LED on the Pixy2
     */
    public SetLEDColor(java.awt.Color color) {
        _color = color;
    }

    private java.awt.Color _color;

    // Called once when the command executes. Since this is an InstantCommand, the
    // method wil only be called once and not continuously. This is good when
    // something needs to be turned on/off or toggled.
    @Override
    protected void initialize() {
        Robot.vision.setLEDColor(_color);
    }

}
