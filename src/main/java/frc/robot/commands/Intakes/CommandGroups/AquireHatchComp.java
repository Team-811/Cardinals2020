/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Intakes.CommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Utility.*;
import frc.robot.commands.Intakes.InstantCommands.*;
import frc.robot.commands.LED.*;

public class AquireHatchComp extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AquireHatchComp() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    //set command led
    addSequential(new Confetti());
    //Ensure that cargo intake is not in the way in the beginning
    addSequential(new BringUpCargoIntake());
    addSequential(new TimerCommand(0.1));
    //Sequence to grab hatch
    addSequential(new ExtendHatchIntake());
    addSequential(new ReleaseHatch());
    addSequential(new TimerCommand(0.2));
    addSequential(new GrabHatch());
    addSequential(new TimerCommand(0.2));
    addSequential(new RetractHatchIntake());
    //set default led
    addSequential(new RedFlame());
  }
}
