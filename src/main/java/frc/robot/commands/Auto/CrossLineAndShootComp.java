/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeStorage.StopIntakeStorage;
import frc.robot.commands.IntakeStorage.ToggleIntakeStorage;
import frc.robot.commands.IntakeStorage.ToggleIntakeStorageReverse;
import frc.robot.commands.Shooter.ToggleKickerAndShooter;
import frc.robot.commands.Utility.TimerCommand;

public class CrossLineAndShootComp extends CommandGroup {
    /**
     * This command moves the robot forward one robot length, allowing it to cross
     * the initiation line. It then moves the robot back against the wall and runs
     * the shooter to shoot the preloaded balls
     */
    public CrossLineAndShootComp() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        // addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        // addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        addSequential(new CrossLine());
        addSequential(new TimerCommand(0.2));
        addSequential(new DriveInches(Constants.ROBOT_LENGTH + Constants.LINE_TO_WALL,0.4));
        addSequential(new TimerCommand(0.2));
        
        addSequential(new ToggleIntakeStorage());
        addSequential(new ToggleKickerAndShooter());

        addSequential(new TimerCommand(6));
        addSequential(new StopIntakeStorage());
        addSequential(new TimerCommand(0.3));
        addSequential(new ToggleIntakeStorageReverse());
        addSequential(new TimerCommand(0.3));
        addSequential(new StopIntakeStorage());
        addSequential(new TimerCommand(0.3));
        addSequential(new ToggleIntakeStorage());

        addSequential(new TimerCommand(3));
        addSequential(new StopIntakeStorage());
        addSequential(new TimerCommand(0.3));
        addSequential(new ToggleIntakeStorageReverse());
        addSequential(new TimerCommand(0.3));
        addSequential(new StopIntakeStorage());
        addSequential(new TimerCommand(0.3));
        addSequential(new ToggleIntakeStorage());
    }
}
