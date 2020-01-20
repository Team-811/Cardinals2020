/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.Drivetrain.DriveWithJoy;
import frc.robot.lib.Output;
import frc.robot.lib.TankDrive;

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

  //Motors
  private CANSparkMax topLeftMotor;
  private CANSparkMax topRightMotor;
  private CANSparkMax bottomLeftMotor;
  private CANSparkMax bottomRightMotor;

  private CANEncoder topLeftEncoder;
  private CANEncoder topRightEncoder;
  private CANEncoder bottomLeftEncoder;
  private CANEncoder bottomRightEncoder;

  private AHRS gyro;

  private TankDrive drivetrain; 

  public Drivetrain()
  {
      topLeftMotor = new CANSparkMax(RobotMap.DRIVE_TOP_LEFT_MOTOR, MotorType.kBrushless);
      topRightMotor = new CANSparkMax(RobotMap.DRIVE_TOP_RIGHT_MOTOR, MotorType.kBrushless);
      bottomLeftMotor = new CANSparkMax(RobotMap.DRIVE_BOTTOM_LEFT_MOTOR, MotorType.kBrushless);
      bottomRightMotor = new CANSparkMax(RobotMap.DRIVE_BOTTOM_RIGHT_MOTOR, MotorType.kBrushless); 
      
      topLeftEncoder = topLeftMotor.getEncoder();
      topRightEncoder = topRightMotor.getEncoder();
      bottomLeftEncoder = bottomLeftMotor.getEncoder();
      bottomRightEncoder = bottomRightMotor.getEncoder();      

      configureSparkMAX();
      zeroEncoders();

      gyro = new AHRS(SerialPort.Port.kMXP);
      gyro.reset();
      invertGyro(false);     

      drivetrain = new TankDrive();                 
  }

  //how fast motors will go; value between 0-1
  private double SpeedScale = 1;

  //drive mode; true = arcade; false = tank
  private boolean DriveMode = false;

  public void DriveWithJoy(double leftStick, double rightStick)
  {
      //Tank Drive
      if(DriveMode == false)
      {
        Output driveOutput;     
      
        driveOutput = drivetrain.tankDrive(leftStick * SpeedScale, rightStick * SpeedScale);     
  
        topLeftMotor.set(driveOutput.getLeftValue());
        bottomLeftMotor.set(driveOutput.getLeftValue());
  
        topRightMotor.set(driveOutput.getRightValue());
        bottomRightMotor.set(driveOutput.getRightValue());
      }

      //Arcade Drive
      else
      {
        Output driveOutput;
        double correction;
    
        if(rightStick < 0.2 && rightStick > -0.2) {
            //when not rotating, compare your current gyro pos to the last time you were rotating to get error
            correction = gyroCorrection();
          } 
        else 
        {
            correction = 0;
        }
        
        if(gyro.isConnected())
          driveOutput = drivetrain.arcadeDrive(leftStick * SpeedScale, (rightStick - correction) * SpeedScale);
        else
          driveOutput = drivetrain.arcadeDrive(leftStick * SpeedScale, rightStick * SpeedScale);     
    
          topLeftMotor.set(driveOutput.getLeftValue());
          bottomLeftMotor.set(driveOutput.getLeftValue());
    
          topRightMotor.set(driveOutput.getRightValue());
          bottomRightMotor.set(driveOutput.getRightValue());
        
        prevAngle = getGyroAngle(); //Stores previous angle
        
      }     

  }

  //toggle slow mode
  public void slowMode(boolean isSlow)
  {
    if(isSlow)
        SpeedScale = 0.5;
    else
        SpeedScale = 1;
  }

  //toggle drive mode
  public void toggleDriveMode()
  {
    DriveMode = !DriveMode;
  }

  public void stopDrivetrain()
  {
     topLeftMotor.set(0);
     bottomLeftMotor.set(0);

     topRightMotor.set(0);
     bottomRightMotor.set(0);
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
    return ((topLeftEncoder.getPosition()+bottomLeftEncoder.getPosition())/2);
  }

  public double getRightEncoder()
  {
    return ((topRightEncoder.getPosition()+bottomRightEncoder.getPosition())/2);
  }
  

  //Encoder Velocities
  public double getLeftVelocity()
  {
    return (topLeftEncoder.getVelocity()+bottomLeftEncoder.getVelocity()) / 2;
  }

  public double getRightVelocity()
  {
    return (topRightEncoder.getVelocity()+bottomRightEncoder.getVelocity()) / 2;
  } 

  public double getForwardVelocity()
  {
    return (getLeftVelocity() + getRightVelocity()) / 2;
  }
  
  //Gyro measurements

  private int gyroInversion = 1;

  public double getGyroAngle()
  {
      return gyroInversion * gyro.getAngle();
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

  public void zeroEncoders()
  {
    topLeftEncoder.setPosition(0);
    topRightEncoder.setPosition(0);
    bottomLeftEncoder.setPosition(0);
    bottomRightEncoder.setPosition(0);
  }
  
  private void configureSparkMAX()
  {    
    zeroEncoders();
    topLeftMotor.setInverted(false);
    topRightMotor.setInverted(false);
    bottomLeftMotor.setInverted(false);
    bottomRightMotor.setInverted(false);

    topLeftMotor.setIdleMode(IdleMode.kBrake);
    topRightMotor.setIdleMode(IdleMode.kBrake);
    bottomLeftMotor.setIdleMode(IdleMode.kBrake);
    bottomRightMotor.setIdleMode(IdleMode.kBrake);
  } 

  @Override
  public void outputSmartdashboard() 
  {
      SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
      SmartDashboard.putNumber("Left Encoder", getLeftEncoder());
      SmartDashboard.putNumber("Right Encoder", getRightEncoder()); 
      SmartDashboard.putNumber("Right Velocity", getRightVelocity());
      SmartDashboard.putNumber("Left Velocity", getLeftVelocity()); 
      
      String mode;
      if(DriveMode)
        mode = "Arcade";
      else
        mode = "Tank";
      SmartDashboard.putString("Drive Mode ", mode);

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
    configureSparkMAX();
  }

  @Override
  public void testSubsystem() 
  {


  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
      setDefaultCommand(new DriveWithJoy());
  }
}
