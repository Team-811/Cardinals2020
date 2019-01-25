/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.Utility.MecanumDrive;
import frc.robot.Utility.Output;
import frc.robot.commands.*;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.AHRSProtocol;
import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX topLeftMotor;
  private TalonSRX topRightMotor;
  private TalonSRX bottomLeftMotor;
  private TalonSRX bottomRightMotor;

  private AHRS gyro;

  private MecanumDrive drivetrain;

  public Drive()
  {
      topLeftMotor = new TalonSRX(Robot.robotMap.DRIVE_TOP_LEFT_MOTOR);
      topRightMotor = new TalonSRX(Robot.robotMap.DRIVE_TOP_RIGHT_MOTOR);
      bottomLeftMotor = new TalonSRX(Robot.robotMap.DRIVE_BOTTOM_LEFT_MOTOR);
      bottomRightMotor = new TalonSRX(Robot.robotMap.DRIVE_BOTTOM_RIGHT_MOTOR);

      topRightMotor.setInverted(true);
      bottomRightMotor.setInverted(true);

      gyro = new AHRS(SerialPort.Port.kMXP);
      gyro.reset();

      drivetrain = new MecanumDrive();
  }


  public void DriveWithJoy(double leftJoy, double rightJoy, double strafe)
  {
      
      boolean quickTurn;

      if(leftJoy <= .2 && leftJoy >= -.2)
      {
          quickTurn = true;
      }
      else
      {
          quickTurn = false;
      }

     /* 
      int inverted;
      Output driveOutput=drivetrain.arcadeMecanumDrive(leftJoy, rightJoy, strafe, 1);
      if(Robot.controllers.operatorController.aButton.get())
      {
            inverted = -1;
            driveOutput = drivetrain.arcadeMecanumDrive(leftJoy, rightJoy, strafe, inverted);
      }
      else if (Robot.controllers.operatorController.bButton.get())
      {
          driveOutput = drivetrain.tankMecanumDrive(leftJoy, rightJoy, strafe, 0.1, 1);
      }
      else if (Robot.controllers.operatorController.xButton.get())
      {
          driveOutput = drivetrain.tankMecanumDrive(leftJoy, rightJoy, strafe, 0.1, -1);
      }
      else if (Robot.controllers.operatorController.yButton.get())
      {
          inverted = 1;  
          driveOutput = drivetrain.RyanarcadeMecanumDrive(leftJoy, rightJoy, strafe, 0.1, inverted);
      }
      */
      

      
      SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
      //Output driveOutput=drivetrain.arcadeMecanumDrive(leftJoy, rightJoy, strafe, 1);
      Output driveOutput = drivetrain.fieldOrientedDrive(leftJoy, strafe, rightJoy, gyro.getAngle());

      topLeftMotor.set(ControlMode.PercentOutput, driveOutput.getTopLeftValue());
      topRightMotor.set(ControlMode.PercentOutput, driveOutput.getTopRightValue());
      bottomLeftMotor.set(ControlMode.PercentOutput, driveOutput.getBottomLeftValue());
      bottomRightMotor.set(ControlMode.PercentOutput, driveOutput.getBottomRightValue());

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
      setDefaultCommand(new DriveWithJoy());
  }
}
