/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;

public class CrossLine extends InstantCommand {
    /**
     * This command moves the robot forward one robot length, allowing it to cross
     * the initiation line
     */
    public CrossLine() {

    }

    // Called once when the command executes. Since this is an InstantCommand, the
    // method wil only be called once and not continuously. This is good when
    // something needs to be turned on/off or toggled.
    @Override
    protected void initialize() {
        // Drive backward a distance equal to the robot length to cross the line
        Robot.drivetrain.driveInches(Constants.ROBOT_LENGTH, -0.6);
    }

}
