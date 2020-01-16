/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.Drivetrain.DriveWithJoy;
import frc.robot.lib.Output;
import frc.robot.lib.TalonChecker;
import frc.robot.lib.TankDrive;
import frc.robot.lib.UnitConverter;

/**
 * This is a subsystem class.  A subsystem interacts with the hardware components on the robot.  This subsystem deals with the 
 * mecanum drivetrain.  The drivetrain includes 4 motors with encoders on each, an encoder on a dead wheel to track strafing, and 
 * a gyro to measure the angle of the robot.  For controlling the subsystem, there are methods that do field oriented driving with
 * gyro correction for joystick control, PID loops for the vision, and Motion Profiling for auto driving.
 */

public class Drivetrain extends Subsystem implements ISubsystem{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Drivetrain instance = new Drivetrain();

  public static Drivetrain getInstance() {
    return instance;
  }

  private TalonSRX leftMotor;
  private TalonSRX rightMotor;

  private AHRS gyro;

  private TankDrive drivetrain; 

  public Drivetrain()
  {
      leftMotor = new TalonSRX(RobotMap.DRIVE_LEFT_MOTOR);
      rightMotor = new TalonSRX(RobotMap.DRIVE_RIGHT_MOTOR);      
      configureTalons();

      gyro = new AHRS(SerialPort.Port.kMXP);
      gyro.reset();
      invertGyro(false);     

      drivetrain = new TankDrive();
                 
  }

  private double SpeedScale = 1;

  public void DriveWithJoy(double forward, double rotation)
  {
      Output driveOutput;
      double correction;

      if(rotation < 0.2 && rotation > -0.2) {
          //when not rotating, compare your current gyro pos to the last time you were rotating to get error
          correction = gyroCorrection();
        } 
      else 
      {
          correction = 0;
      }
      
      if(gyro.isConnected())
        driveOutput = drivetrain.arcadeMecanumDrive(forward * SpeedScale, (rotation - correction) * SpeedScale);
      else
        driveOutput = drivetrain.arcadeMecanumDrive(forward * SpeedScale, rotation * SpeedScale);     

      leftMotor.set(ControlMode.PercentOutput, driveOutput.getLeftValue() * Constants.LEFT_COEFFICIENT);
      rightMotor.set(ControlMode.PercentOutput, driveOutput.getRightValue() * Constants.RIGHT_COEFFICENT);     
      
      prevAngle = getGyroAngle(); //Stores previous angle

  }

  public void slowMode(boolean isSlow)
  {
    if(isSlow)
        SpeedScale = 0.5;
    else
        SpeedScale = 1;
  }

  public void stopDrivetrain()
  {
    leftMotor.set(ControlMode.PercentOutput, 0);
    rightMotor.set(ControlMode.PercentOutput, 0);
  }

  //Gyro Correction

  private double gyroCorrectRate = 0.1;

  private double prevAngle;

  public double gyroCorrection()
  {
      return (getGyroAngle() - prevAngle) * gyroCorrectRate;
  }


  //Encoder Distances

  public double getLeftEncoder()
  {
    return UnitConverter.ticksToMeters(leftMotor.getSelectedSensorPosition(), 1378, Constants.wheelDiameter);
  }

  public double getRightEncoder()
  {
    return UnitConverter.ticksToMeters(rightMotor.getSelectedSensorPosition(), 1373, Constants.wheelDiameter);
  }

  public void zeroEncoders()
  {
    leftMotor.setSelectedSensorPosition(0);
    rightMotor.setSelectedSensorPosition(0);
  }

  //Encoder Velocities

  public double getLeftVelocity()
  {
    return UnitConverter.talonUnitsToMetersPerSecond(leftMotor.getSelectedSensorVelocity(), Constants.ticksPerRotation, Constants.wheelDiameter);
  }

  public double getRightVelocity()
  {
    return UnitConverter.talonUnitsToMetersPerSecond(rightMotor.getSelectedSensorVelocity(), Constants.ticksPerRotation, Constants.wheelDiameter);
  } 

  public double getForwardVelocity()
  {
    return (getLeftVelocity() + getRightVelocity()) / 2;
  }
  
  //Gyro measurements

  private int gyroInversion = 1;

  public double getGyroAngle()
  {
      return gyroInversion *gyro.getAngle();
  }

  public double getAngularVelocity()
  {
      return gyroInversion * gyro.getRate();
  }

  public void invertGyro(boolean inverted)
  {
      if(inverted)
        gyroInversion = -1;
      else
        gyroInversion = 1;
  }

  public void zeroGyro()
  {
      gyro.zeroYaw();
  }

  private void configureTalons()
  {
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    leftMotor.setSensorPhase(false);
    rightMotor.setSensorPhase(false);    

    leftMotor.setSelectedSensorPosition(0);
    rightMotor.setSelectedSensorPosition(0);

    leftMotor.configNominalOutputForward(0);
    rightMotor.configNominalOutputReverse(0);
    
  } 

  @Override
  public void outputSmartdashboard() 
  {
      SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
      SmartDashboard.putNumber("Left Encoder", getLeftEncoder());
      SmartDashboard.putNumber("Right Encoder", getRightEncoder()); 
      SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
  }

  @Override
  public void zeroSensors() 
  {
    zeroEncoders();
    zeroGyro();
  }

  @Override
  public void resetSubsystem() 
  {
    stopDrivetrain();
    zeroSensors();
    configureTalons();
  }

  @Override
  public void testSubsystem() {
    boolean success = true;
    
    System.out.println("///////////////////////////////////////////////////");
    System.out.println("***************Beginning Drivetrain Test***************");
    Timer.delay(0.2);

    //Test left motor
    System.out.println("Testing Left Motor and Encoder");
    Timer.delay(0.5);
    TalonChecker checker = new TalonChecker("Left Wheel Talon", leftMotor, false);
    success = checker.runTest(5, 75);
    Timer.delay(0.2);

    if(!success)
    {
        System.out.println("***************Error in Left Motor or Encoder***************");
        return;
    }

    //Test right motor
    System.out.println("Testing Right Motor and Encoder");
    Timer.delay(0.5);
    checker = new TalonChecker("Right Wheel Talon", rightMotor, false);
    success = checker.runTest(2, 75);
    Timer.delay(0.2);

    if(!success)
    {
        System.out.println("***************Error in Right Motor or Encoder***************");
        return;
    }
  
    System.out.println("Testing Gyro");
    Timer.delay(0.5);
    success = gyro.isConnected();
    Timer.delay(0.2);

    if(!success)
    {
        System.out.println("***************Gyro Not Connected***************");
        return;
    }

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
      setDefaultCommand(new DriveWithJoy());
  }
}
