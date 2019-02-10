/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.MecanumDrive;
import frc.robot.lib.MotionProfiling;
import frc.robot.lib.Output;
import frc.robot.lib.PIDController;
import frc.robot.lib.TalonChecker;
import frc.robot.lib.UnitConverter;
import jaci.pathfinder.Waypoint;
import frc.robot.commands.Drivetrain.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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
  private PIDController pid;
  private MotionProfiling motionProfile;

  private Encoder strafeEncoder;
  
  private int kP;
  private int kI;
  private int kD;
  private int kF;
  private int kTimeoutMs;

  public Drivetrain()
  {
      topLeftMotor = new TalonSRX(RobotMap.DRIVE_TOP_LEFT_MOTOR);
      topRightMotor = new TalonSRX(RobotMap.DRIVE_TOP_RIGHT_MOTOR);
      bottomLeftMotor = new TalonSRX(RobotMap.DRIVE_BOTTOM_LEFT_MOTOR);
      bottomRightMotor = new TalonSRX(RobotMap.DRIVE_BOTTOM_RIGHT_MOTOR);

      strafeEncoder = new Encoder(RobotMap.DRIVE_STRAFE_ENCODER_ACHANNEL, RobotMap.DRIVE_STRAFE_ENCODER_BCHANNEL);

      configureTalons();

      gyro = new AHRS(SerialPort.Port.kMXP);
      gyro.reset();

      drivetrain = new MecanumDrive();
      motionProfile = new MotionProfiling(Constants.maxVelocity, Constants.maxAcceleration, Constants.maxJerk, Constants.wheelbase);
      pid = new PIDController(kP, kI, kD);

      drivetrain.invertForwardBackward(true);
      drivetrain.invertStrafing(true);
  }


  public void DriveWithJoy(double leftJoy, double rightJoy, double strafe)
  {
      Output driveOutput;

      if(gyro.isConnected())
      {
        driveOutput = drivetrain.fieldOrientedDrive(leftJoy, strafe, rightJoy, getInvertedGyroAngle());
      }
      else
        driveOutput = drivetrain.arcadeMecanumDrive(leftJoy, rightJoy, strafe);
      

      topLeftMotor.set(ControlMode.PercentOutput, driveOutput.getTopLeftValue());
      topRightMotor.set(ControlMode.PercentOutput, driveOutput.getTopRightValue());
      bottomLeftMotor.set(ControlMode.PercentOutput, driveOutput.getBottomLeftValue());
      bottomRightMotor.set(ControlMode.PercentOutput, driveOutput.getBottomRightValue());

  }

  public void loadTrajectory(Waypoint[] path, boolean reverse)
  {
      motionProfile.loadTrajectory(path, reverse);
  }

  public void followTrajectory(boolean reverse)
  {

    double leftEncoderMeters = getLeftEncoder();
    double rightEncoderMeters = getRightEncoder();

    Output driveOutput = motionProfile.getNextDriveSignal(reverse, topLeftMotor.getSelectedSensorPosition(), topRightMotor.getSelectedSensorPosition(), gyro.getAngle(), false);

    double velocityLeft = UnitConverter.metersPerSecondToTalonUnits(driveOutput.getLeftValue(), Constants.wheelDiameter, Constants.ticksPerRotation);
    double velocityRight = UnitConverter.metersPerSecondToTalonUnits(driveOutput.getRightValue(), Constants.wheelDiameter, Constants.ticksPerRotation);

    setVelocity(velocityLeft, velocityRight);
  }

  public void setVelocity(double velocityLeft, double velocityRight)
  {
    topLeftMotor.set(ControlMode.Velocity, velocityLeft);
    topRightMotor.set(ControlMode.Velocity, velocityRight);
    bottomLeftMotor.set(ControlMode.Follower, topLeftMotor.getDeviceID());
    bottomLeftMotor.set(ControlMode.Follower, topRightMotor.getDeviceID());

  }

  public void rotateToAngle(double goalAngleInDegrees)
  {
      double rotation = pid.updatePID(getGyroAngle(), goalAngleInDegrees);
      Output driveOutput = drivetrain.arcadeMecanumDrive(0, rotation, 0);

      topLeftMotor.set(ControlMode.PercentOutput, driveOutput.getTopLeftValue());
      topRightMotor.set(ControlMode.PercentOutput, driveOutput.getTopRightValue());
      bottomLeftMotor.set(ControlMode.PercentOutput, driveOutput.getBottomLeftValue());
      bottomRightMotor.set(ControlMode.PercentOutput, driveOutput.getBottomRightValue());


  }

  public void strafeToSetpoint(double goalInMeters)
  {
    double rotation = pid.updatePID(getGyroAngle(), goalInMeters);
    Output driveOutput = drivetrain.arcadeMecanumDrive(0, rotation, 0);

    topLeftMotor.set(ControlMode.PercentOutput, driveOutput.getTopLeftValue());
    topRightMotor.set(ControlMode.PercentOutput, driveOutput.getTopRightValue());
    bottomLeftMotor.set(ControlMode.PercentOutput, driveOutput.getBottomLeftValue());
    bottomRightMotor.set(ControlMode.PercentOutput, driveOutput.getBottomRightValue());
  }


  //Encoder Distances

  public double getTopLeftEncoder()
  {
    return UnitConverter.ticksToMeters(topLeftMotor.getSelectedSensorPosition(), Constants.ticksPerRotation, Constants.wheelDiameter);
  }

  public double getTopRightEncoder()
  {
    return UnitConverter.ticksToMeters(topRightMotor.getSelectedSensorPosition(), Constants.ticksPerRotation, Constants.wheelDiameter);
  }

  public double getBottomLeftEncoder()
  {
    return UnitConverter.ticksToMeters(bottomLeftMotor.getSelectedSensorPosition(), Constants.ticksPerRotation, Constants.wheelDiameter);
  }

  public double getBottomRightEncoder()
  {
    return UnitConverter.ticksToMeters(bottomRightMotor.getSelectedSensorPosition(), Constants.ticksPerRotation, Constants.wheelDiameter);
  }

  public double getLeftEncoder()
  {
    return (getTopLeftEncoder() + getBottomLeftEncoder()) / 2;
  }

  public double getRightEncoder()
  {
    return (getTopRightEncoder() + getBottomRightEncoder()) / 2;
  }

  public double getStrafeEncoder()
  {
    return UnitConverter.ticksToMeters(strafeEncoder.getDistance(), Constants.ticksPerRotation, Constants.smallWheelDiameter);
  }




  //Encoder Velocities

  public double getTopLeftVelocity()
  {
    return UnitConverter.talonUnitsToMetersPerSecond(topLeftMotor.getSelectedSensorVelocity(), Constants.ticksPerRotation, Constants.wheelDiameter);
  }

  public double getTopRightVelocity()
  {
    return UnitConverter.talonUnitsToMetersPerSecond(topRightMotor.getSelectedSensorVelocity(), Constants.ticksPerRotation, Constants.wheelDiameter);
  }

  public double getBottomLeftVelocity()
  {
    return UnitConverter.talonUnitsToMetersPerSecond(bottomLeftMotor.getSelectedSensorVelocity(), Constants.ticksPerRotation, Constants.wheelDiameter);
  }

  public double getBottomRightVelocity()
  {
    return UnitConverter.talonUnitsToMetersPerSecond(bottomRightMotor.getSelectedSensorVelocity(), Constants.ticksPerRotation, Constants.wheelDiameter);
  }

  public double getLeftVelocity()
  {
    return (getTopLeftVelocity() + getBottomLeftVelocity()) / 2;
  }

  public double getRightVelocity()
  {
    return (getTopRightVelocity() + getBottomRightVelocity()) / 2;
  }

  public double getStrafeVelocity()
  {
    //Divided by 10 because getRate time measurement is in seconds and not 100ms
    return UnitConverter.talonUnitsToMetersPerSecond(strafeEncoder.getRate(), Constants.ticksPerRotation, Constants.smallWheelDiameter) / 10;
  }






  //Gyro angles

  public double getGyroAngle()
  {
      return gyro.getAngle();
  }

  public double getInvertedGyroAngle()
  {
      return -gyro.getAngle();
  }







  private void configureTalons()
  {
    topLeftMotor.setInverted(false);
    topRightMotor.setInverted(false);
    bottomLeftMotor.setInverted(false);
    bottomRightMotor.setInverted(true);

    topLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    topRightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    bottomLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    bottomRightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    topLeftMotor.setSensorPhase(true);
    topRightMotor.setSensorPhase(true);
    bottomLeftMotor.setSensorPhase(true);
    bottomRightMotor.setSensorPhase(true);

    bottomLeftMotor.setSelectedSensorPosition(0);
    bottomRightMotor.setSelectedSensorPosition(0);

    bottomLeftMotor.configNominalOutputForward(0);
    bottomLeftMotor.configNominalOutputReverse(0);
    bottomLeftMotor.configPeakOutputForward(1);
    bottomLeftMotor.configPeakOutputReverse(-1);
  }


  public void zeroEncoders()
  {
    topLeftMotor.setSelectedSensorPosition(0);
    topRightMotor.setSelectedSensorPosition(0);
    bottomRightMotor.setSelectedSensorPosition(0);
    bottomRightMotor.setSelectedSensorPosition(0);
  }

  public void zeroGyro()
  {
      gyro.zeroYaw();
  }


  @Override
  public void outputSmartdashboard() 
  {
    
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
    topLeftMotor.set(ControlMode.PercentOutput, 0);
    topRightMotor.set(ControlMode.PercentOutput, 0);
    bottomLeftMotor.set(ControlMode.PercentOutput, 0);
    bottomRightMotor.set(ControlMode.PercentOutput, 0);

    zeroSensors();
    configureTalons();
  }

  @Override
  public void testSubsystem() {
    boolean sucess = true;

    Timer delay = new Timer();
    System.out.println("///////////////////////////////////////////////////");
    System.out.println("***************Beginning Drivetrain Test***************");
    Timer.delay(0.2);


    //Test top left motor
    System.out.println("Testing Top Left Motor and Encoder");
    Timer.delay(0.5);
    TalonChecker checker = new TalonChecker("Top Left Wheel Talon", topLeftMotor, false);
    sucess = checker.runTest(5, 0); //TODO
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Error in Top Left Motor or Encoder***************");
        return;
    }

    //Test top right motor
    System.out.println("Testing Top Right Motor and Encoder");
    Timer.delay(0.5);
    checker = new TalonChecker("Top Right Wheel Talon", topLeftMotor, false);
    sucess = checker.runTest(5, 0); //TODO
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Error in Top Right Motor or Encoder***************");
        return;
    }

    //Test bottom left motor
    System.out.println("Testing Bottom Left Motor and Encoder");
    Timer.delay(0.5);
    checker = new TalonChecker("Bottom Left Wheel Talon", topLeftMotor, false);
    sucess = checker.runTest(5, 0); //TODO
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Error in Bottom Left Motor or Encoder***************");
        return;
    }

    //Test bottom right motor
    System.out.println("Testing Bottom Right Motor and Encoder");
    Timer.delay(0.5);
    checker = new TalonChecker("Bottom Right Wheel Talon", topLeftMotor, false);
    sucess = checker.runTest(5, 0); //TODO
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Error in Bottom Right Motor or Encoder***************");
        return;
    }

    System.out.println("Testing Gyro");
    Timer.delay(0.5);
    sucess = gyro.isConnected();
    Timer.delay(0.2);

    if(!sucess)
    {
        System.out.println("***************Gyro Not Connected***************");
        return;
    }


    if(sucess)
        System.out.println("***************Everything in Drivetrain is working***************");
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
