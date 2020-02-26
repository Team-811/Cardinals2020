/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AlignLargestTarget extends Command {

    /**
     * This command attemps to line up with and grab the ball it can see the most
     * clearly
     */
    public AlignLargestTarget() {
    }

    // attempt to drive toward the ball while a button is held down
    @Override
    protected void execute() {
        Robot.vision.alignLargestTarget();
    }

    // stop lining up when the button is released
    @Override
    protected void end() {
        Robot.drivetrain.stopDrivetrain();
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
