/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

class DriveInches extends InstantCommand {
    /**
     * This command drives the robot forward a given number of inches. It can only
     * be used within the "auto" package as more of a utility command
     */

    private double _inches;
    private double _speed;

    public DriveInches(double inches, double speed) {
        _inches = inches;
        _speed = speed;
    }

    // Called once when the command executes. Since this is an InstantCommand, the
    // method wil only be called once and not continuously. This is good when
    // something needs to be turned on/off or toggled.
    @Override
    protected void initialize() {
        Robot.drivetrain.driveInches(_inches, _speed);
    }

}
