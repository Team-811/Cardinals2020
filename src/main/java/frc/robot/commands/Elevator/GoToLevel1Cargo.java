/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator.Positions;

/**
 * Add your docs here.
 */
public class GoToLevel1Cargo extends InstantCommand {
  /**
   * Add your docs here.
   */
  public GoToLevel1Cargo() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.elevator);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.elevator.setPosition(Positions.Level1Cargo.getPosition());
  }

}
