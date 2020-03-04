/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ColorWheel;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;

public class RotationControlTimed extends InstantCommand {

    /**
     * This command spins the color wheel for a given time
     */
    public RotationControlTimed(double time) {
        _time = time;
    }

    private double _time = 0;

    // Called once when the command executes. Since this is an InstantCommand, the
    // method wil only be called once and not continuously. This is good when
    // something needs to be turned on/off or toggled.
    @Override
    protected void initialize() {
        Robot.colorWheel.rotationControlTimed(_time, Constants.COLOR_WHEEL_SPEED);
    }

}
