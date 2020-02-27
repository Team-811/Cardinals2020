/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class RunTelescopeWinch extends Command {

    /**
     * This command runs the motor to extend the arm perpendicular to the ground
     * while a button is pressed
     */
    public RunTelescopeWinch() {
    }

    // run the motor while a button is pressed
    @Override
    protected void execute() {
        Robot.climber.runTelescopeWinch(Constants.TELESCOPE_WINCH_SPEED);
    }

    // stop the arm winch when the command is no longer being called
    @Override
    protected void end() {
        Robot.climber.runTelescopeWinch(0);
        Robot.setDefaultLED();
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