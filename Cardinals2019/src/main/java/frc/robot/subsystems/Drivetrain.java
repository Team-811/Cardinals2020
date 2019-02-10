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
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.MecanumDrive;
import frc.robot.lib.MotionProfiling;
import frc.robot.lib.Output;
import frc.robot.lib.PIDController;
import frc.robot.lib.UnitConverter;
import jaci.pathfinder.Waypoint;
import frc.robot.commands.Drivetrain.*;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.AHRSProtocol;
import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
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
      motionProfile = new MotionProfiling(1, 1, 60, Constants.wheelbase);

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

  public void loadTrajectory(Waypoint[] path)
  {
      motionProfile.loadTrajectory(path);
  }

  public void followTrajectory(boolean reverse)
  {

    double leftEncoderMeters = encoderTicksToMeters(bottomLeftMotor.getSelectedSensorPosition(), wheelDiameter);
    double rightEncoderMeters = encoderTicksToMeters(bottomRightMotor.getSelectedSensorPosition(), wheelDiameter);

    Output driveOutput = motionProfile.getNextDriveSignal(reverse, topLeftMotor.getSelectedSensorPosition(), topRightMotor.getSelectedSensorPosition(), gyro.getAngle(), false);

    double velocityLeft = metersPerSecondToUnitsPer100MS(driveOutput.getLeftValue(), wheelDiameter);
    double velocityRight = metersPerSecondToUnitsPer100MS(driveOutput.getLeftValue(), wheelDiameter);

    topLeftMotor.set(ControlMode.Velocity, velocityLeft);
    topRightMotor.set(ControlMode.Velocity, velocityRight);
    bottomLeftMotor.set(ControlMode.Velocity, velocityLeft);
    bottomRightMotor.set(ControlMode.Velocity, velocityRight);
  }

  public void setVelocity(double velocityInMetersPerSec)
  {
    double velocityLeft = metersPerSecondToUnitsPer100MS(velocityInMetersPerSec, wheelDiameter);
    double velocityRight = metersPerSecondToUnitsPer100MS(velocityInMetersPerSec, wheelDiameter);

    topLeftMotor.set(ControlMode.Velocity, velocityLeft);
    topRightMotor.set(ControlMode.Velocity, velocityRight);
    bottomLeftMotor.set(ControlMode.Velocity, velocityLeft);
    bottomRightMotor.set(ControlMode.Velocity, velocityRight);
  }

  public double getTopLeftEncoder()
  {
    return topLeftMotor.getSelectedSensorPosition();
  }

  public double getTopRightEncoder()
  {
    return topRightMotor.getSelectedSensorPosition();
  }

  public double getLeftEncoder()
  {
    return (getTopLeftEncoder() + getBottomLeftEncoder()) / 2;
  }

  public double getRightEncoder()
  {
    return (getTopRightEncoder() + getBottomRightEncoder()) / 2;
  }

  public double getBottomLeftEncoder()
  {
    return bottomLeftMotor.getSelectedSensorPosition();
  }

  public double getBottomRightEncoder()
  {
    return bottomRightMotor.getSelectedSensorPosition();
  }

  public double getStrafeEncoder()
  {
    return strafeEncoder.getDistance();
  }

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
  public void checkSubsystem()
  {
    
  }

  @Override
  public void testSubsystem() {
    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
      setDefaultCommand(new DriveWithJoy());
  }
}
