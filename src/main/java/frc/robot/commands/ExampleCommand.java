/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

public class ExampleCommand extends Command {
    /**
     * This is the template for a command
     */
    public ExampleCommand() {

    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run. This is usually
    // where the method for the subsystem will be called. Be aware this continuously
    // calls the method and treats it as a loop. A command is normally used when
    // information in methods needs to be updated each time through. Commands are
    // typically called while a button on the controller is held down rather than
    // clicked
    @Override
    protected void execute() {

    }

    // If this returns true for any reason, the command will stop, even if the
    // command is still being called by a button/other method
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true. Put what should happen when the
    // button on the controller is released (for example, stopping the spinning of a
    // motor)
    @Override
    protected void end() {

    }

    // if the command is interrupted, run the same thing as if it had been ended
    @Override
    protected void interrupted() {
        end();
    }
}
