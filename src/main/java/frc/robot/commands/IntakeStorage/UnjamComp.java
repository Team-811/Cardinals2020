/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.IntakeStorage;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.Utility.TimerCommand;

public class UnjamComp extends CommandGroup {
    /**
     * This is a command. A command is used to make actual actions happen on the
     * robot. It can be a single action or a sequence of actions. This one is a
     * combination of commands that runs the intake/storage back and forth a few
     * times to try to unjam any stuck balls
     */
    public UnjamComp() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        // addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        // addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // if the intake was running when unjam is called, turn it back on after
        // unjamming. Else, just run it backwards and stop
        
        addSequential(new StopIntakeStorage());
        addSequential(new ToggleIntakeStorageReverse());
        addSequential(new TimerCommand(0.5));
        addSequential(new StopIntakeStorage());      

       
        //addSequential(new ToggleIntakeStorage());

    }
}
