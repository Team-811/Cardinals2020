/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.lib.MecanumDrive;
import frc.robot.lib.Output;
//import frc.robot.lib.TalonChecker;
import frc.robot.commands.Drivetrain.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

/**
 * This is a subsystem class.  A subsystem interacts with the hardware components on the robot.
 */
public class Drivetrain extends Subsystem implements ISubsystem{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX topLeftMotor;
  private TalonSRX topRightMotor;
  private TalonSRX bottomLeftMotor;
  private TalonSRX bottomRightMotor;

  private AHRS gyro;

  private MecanumDrive drivetrain;

  public Drivetrain()
  {
      topLeftMotor = new TalonSRX(RobotMap.DRIVE_TOP_LEFT_MOTOR);
      topRightMotor = new TalonSRX(RobotMap.DRIVE_TOP_RIGHT_MOTOR);
      bottomLeftMotor = new TalonSRX(RobotMap.DRIVE_BOTTOM_LEFT_MOTOR);
      bottomRightMotor = new TalonSRX(RobotMap.DRIVE_BOTTOM_RIGHT_MOTOR);

      //topRightMotor.setInverted(false);
      bottomRightMotor.setInverted(true);


      gyro = new AHRS(RobotMap.GYRO_PORT);
      gyro.reset();

      drivetrain = new MecanumDrive();

      drivetrain.invertForwardBackward(true);
      drivetrain.invertStrafing(true);
  }


  public void DriveWithJoy(double leftJoy, double rightJoy, double strafe)
  {
      SmartDashboard.putNumber("Gyro Angle", -1 * gyro.getAngle());
      Output driveOutput=drivetrain.arcadeMecanumDrive(leftJoy, rightJoy, strafe);
      //Output driveOutput = drivetrain.fieldOrientedDrive(leftJoy, strafe, rightJoy, -1 * gyro.getAngle());

      topLeftMotor.set(ControlMode.PercentOutput, driveOutput.getTopLeftValue());
      topRightMotor.set(ControlMode.PercentOutput, driveOutput.getTopRightValue());
      bottomLeftMotor.set(ControlMode.PercentOutput, driveOutput.getBottomLeftValue());
      bottomRightMotor.set(ControlMode.PercentOutput, driveOutput.getBottomRightValue());

  }


  @Override
  public void outputSmartdashboard() 
  {
    
  }

  @Override
  public void zeroSensors() 
  {
    
  }

  @Override
  public void resetSubsystem() 
  {
    
  }

  @Override
  public void checkSubsystem()
  {
    
  }

  @Override
  public void testSubsystem() {
    boolean sucess = true;

    Timer delay = new Timer();
    System.out.println("///////////////////////////////////////////////////");
    System.out.println("***************Beginning Drivetrain Test***************");
    Timer.delay(0.2);


    //Test top left wheel
    System.out.println("Testing Top Left Wheel");
    Timer.delay(0.5);
    //TalonChecker checker = new TalonChecker("Top Left Wheel Talon", topLeftMotor, false);
    //sucess = checker.runTest(5, 0); //TODO
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Error in Top Left Wheel***************");
        return;
    }

    //Test top right wheel
    System.out.println("Testing Top Right Wheel");
    Timer.delay(0.5);
    //checker = new TalonChecker("Top Right Wheel Talon", topRightMotor, false);
    //sucess = checker.runTest(5, 0); //TODO
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Error in Top Right Wheel***************");
        return;
    }

    //Test bottom left wheel
    System.out.println("Testing Bottom Left Wheel");
    Timer.delay(0.5);
    //checker = new TalonChecker("Bottom Left Wheel Talon", bottomLeftMotor, false);
    //sucess = checker.runTest(5, 0); //TODO
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Error in Bottom Left Wheel***************");
        return;
    }

    //Test bottom right wheel
    System.out.println("Testing Bottom Right Motor");
    Timer.delay(0.5);
    //checker = new TalonChecker("Cargo Talon", bottomRightMotor, false);
    //sucess = checker.runTest(5, 0); //TODO
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Error in Bottom Right Wheel***************");
        return;
    }

    //Test Gyro Presence
    System.out.println("Testing Gyro Presence");
    Timer.delay(0.5);
    sucess = gyro.isConnected();
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Error in Gyro***************");
        return;
    }

    if(sucess)
        System.out.println("***************Everything in drivetrain is working***************");
    else
        System.out.println("***************Error in Drivetrain***************");
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
      setDefaultCommand(new DriveWithJoy());
  }
}
