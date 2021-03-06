/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Utility;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ZeroSensors extends InstantCommand {

  /**
   * This command zeros sensors in each of the subsystems such as encoders, gyro,
   * etc
   */
  public ZeroSensors() {

  }

  // Called once when the command executes. Since this is an InstantCommand, the
  // method will only be called once and not continuously. This is good when
  // something needs to be turned on/off or toggled.
  @Override
  protected void initialize() {
    Robot.drivetrain.zeroSensors();
    Robot.shooter.zeroSensors();
    Robot.intakeStorage.zeroSensors();
    Robot.colorWheel.zeroSensors();
  }

}
