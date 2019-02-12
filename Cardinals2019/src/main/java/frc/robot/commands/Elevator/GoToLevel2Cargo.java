/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Add your docs here.
 */
public class GoToLevel2Cargo extends InstantCommand {
  /**
   * Add your docs here.
   */
  public GoToLevel2Cargo() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.Elevator);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.Elevator.setDesiredPosition(Level2Cargo.getposition);
  }

}
