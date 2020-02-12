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
 * This subsystem deals with the drivetrain. The drivetrain includes 4 NEO
 * motors with encoders on each and a gyro to measure the angle of the robot.
 */
public class Drivetrain extends Subsystem implements ISubsystem {

  private static Drivetrain instance = new Drivetrain();

  /**
   * @return A new instance of the Drivetrain
   */
  public static Drivetrain getInstance() {
    return instance;
  }

  // Motors
  private CANSparkMax topLeftMotor;
  private CANSparkMax topRightMotor;
  private CANSparkMax bottomLeftMotor;
  private CANSparkMax bottomRightMotor;

  // Encoders
  private CANEncoder topLeftEncoder;
  private CANEncoder topRightEncoder;
  private CANEncoder bottomLeftEncoder;
  private CANEncoder bottomRightEncoder;

  private AHRS gyro;

  private TankDrive drivetrain;
  private Output driveOutput;

  /**
   * @return A new instance of the Drivetrain
   */
  public Drivetrain() {
    topLeftMotor = new CANSparkMax(RobotMap.DRIVE_TOP_LEFT_MOTOR, MotorType.kBrushless);
    topRightMotor = new CANSparkMax(RobotMap.DRIVE_TOP_RIGHT_MOTOR, MotorType.kBrushless);
    bottomLeftMotor = new CANSparkMax(RobotMap.DRIVE_BOTTOM_LEFT_MOTOR, MotorType.kBrushless);
    bottomRightMotor = new CANSparkMax(RobotMap.DRIVE_BOTTOM_RIGHT_MOTOR, MotorType.kBrushless);

    topLeftEncoder = topLeftMotor.getEncoder();
    topRightEncoder = topRightMotor.getEncoder();
    bottomLeftEncoder = bottomLeftMotor.getEncoder();
    bottomRightEncoder = bottomRightMotor.getEncoder();

    gyro = new AHRS(SerialPort.Port.kMXP);
    gyro.reset();
    invertGyro(false);

    resetSubsystem();

    drivetrain = new TankDrive();
  }

  /**
   * Default speed drivetrain will go; between 0-1
   */
  private double DefaultSpeedScale = 1;

  /**
   * Default speed drivetrain will go during slow mode; between 0-1
   */
  private double SlowSpeedScale = 0.5;

  /**
   * Current speed scale for the robot
   */
  private double SpeedScale = DefaultSpeedScale;

  /**
   * Determines if the robot is currently in slow mode.
   */
  private boolean isSlow = false;

  /**
   * Sets acceleration rate of drivetrain to correct "jerkiness". How many seconds it will take to go from 0 to full speed.
   */
  private double RampRate = 0.4;

  /**
   * True: arcade; False: tank. Set default here.
   */
  private boolean DriveMode = true;

  /**
   * This is the main method for driving the robot. It determines which drive mode
   * is currently selected and outputs to the motors accordingly. For tank drive
   * mode, the left stick controls the left wheels and the right stick controls
   * the right wheels. In arcade mode, the left stick controls forward/reverse
   * motion and the right stick controls turning. Arcade mode has gyro correction
   * to keep the robot moving in a straight line.
   * 
   * @param leftStick  : Y value of left XBox joystick.
   * @param rightStick : Y value of right XBox joystick.
   * @param rotation   : X value of right XBox joystick.
   */
  public void driveWithJoy(double leftStick, double rightStick, double rotation) {
    // Tank Drive
    if (!DriveMode) {
      driveOutput = drivetrain.tankDrive(leftStick * SpeedScale, rightStick * SpeedScale);

      topLeftMotor.set(driveOutput.getLeftValue());
      bottomLeftMotor.set(driveOutput.getLeftValue());

      topRightMotor.set(driveOutput.getRightValue());
      bottomRightMotor.set(driveOutput.getRightValue());
    }

    // Arcade Drive
    else {
      // double correction;

      // if (rotation < 0.2 && rotation > -0.2) {
      // // when not rotating, compare your current gyro pos to the last time you were
      // // rotating to get error
      // correction = gyroCorrection();
      // } else {
      // correction = 0;
      // }

      // if (gyro.isConnected())
      // driveOutput = drivetrain.arcadeDrive(leftStick * SpeedScale, (rotation -
      // correction) * SpeedScale);
      // else

      driveOutput = drivetrain.arcadeDrive(leftStick * SpeedScale, rotation * SpeedScale);

      topLeftMotor.set(driveOutput.getLeftValue());
      bottomLeftMotor.set(driveOutput.getLeftValue());

      topRightMotor.set(-driveOutput.getRightValue());
      bottomRightMotor.set(-driveOutput.getRightValue());

      prevAngle = getGyroAngle(); // Stores previous angle
    }
  }

  /**
   * Toggle slow mode. Robot will be in slow mode as this method is repeatedly
   * called.
   * 
   * @param slow : set this to true for slow mode
   */
  public void slowMode(boolean slow) {
    if (slow) {
      isSlow = true;
      SpeedScale = SlowSpeedScale;
    } else {
      isSlow = false;
      SpeedScale = DefaultSpeedScale;
    }

  }

  /**
   * Toggles the drive mode between arcade/tank
   */
  public void toggleDriveMode() {
    DriveMode = !DriveMode;
  }

  /**
   * 
   * @return Number of rotations of left motor
   */
  public double getLeftEncoder() {
    return (topLeftEncoder.getPosition() + bottomLeftEncoder.getPosition()) / 2;
  }

  /**
   * 
   * @return Number of rotations of right motor
   */
  public double getRightEncoder() {
    return (topRightEncoder.getPosition() + bottomRightEncoder.getPosition()) / -2;
  }

  /**
   * 
   * @return Average of left and right distances
   */
  public double getForwardDistance() {
    return (getLeftEncoder() + getRightEncoder()) / 2;
  }

  /**
   * 
   * @return Left side speed
   */
  public double getLeftVelocity() {
    return (topLeftEncoder.getVelocity() + bottomLeftEncoder.getVelocity()) / 2;
  }

  /**
   * 
   * @return Right side speed
   */
  public double getRightVelocity() {
    return (topRightEncoder.getVelocity() + bottomRightEncoder.getVelocity()) / -2;
  }

  /**
   * 
   * @return Average forward speed
   */
  public double getForwardVelocity() {
    return (getLeftVelocity() + getRightVelocity()) / 2;
  }

  /**
   * Stops the drivetrain
   */
  public void stopDrivetrain() {
    topLeftMotor.set(0);
    bottomLeftMotor.set(0);

    topRightMotor.set(0);
    bottomRightMotor.set(0);
  }

  /**
   * Reset drivetrain encoder distances to 0
   */
  public void zeroEncoders() {
    topLeftEncoder.setPosition(0);
    topRightEncoder.setPosition(0);
    bottomLeftEncoder.setPosition(0);
    bottomRightEncoder.setPosition(0);
  }

  /**
   * Configures subsytem-specific settings for motor controllers
   */
  private void configureMotorControllers() {
    zeroEncoders();
    topLeftMotor.setInverted(false);
    topRightMotor.setInverted(false);
    bottomLeftMotor.setInverted(false);
    bottomRightMotor.setInverted(false);

    topLeftMotor.setIdleMode(IdleMode.kBrake);
    topRightMotor.setIdleMode(IdleMode.kBrake);
    bottomLeftMotor.setIdleMode(IdleMode.kBrake);
    bottomRightMotor.setIdleMode(IdleMode.kBrake);

    topLeftMotor.setOpenLoopRampRate(RampRate);
    topRightMotor.setOpenLoopRampRate(RampRate);
    bottomLeftMotor.setOpenLoopRampRate(RampRate);
    bottomRightMotor.setOpenLoopRampRate(RampRate);
  }

  /**
   * Outputs Drivetrain subsystem information to SmartDashboard for drivers.
   */
  @Override
  public void outputSmartdashboard() {
    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
    SmartDashboard.putNumber("Left Encoder", getLeftEncoder());
    SmartDashboard.putNumber("Right Encoder", getRightEncoder());
    SmartDashboard.putNumber("Forward Distance", getForwardDistance());
    SmartDashboard.putNumber("Right Velocity", getRightVelocity());
    SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Forward Velocity", getForwardVelocity());

    SmartDashboard.putBoolean("Slow Mode", isSlow);

    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());

    String mode;
    if (DriveMode)
      mode = "Arcade";
    else
      mode = "Tank";
    SmartDashboard.putString("Drive Mode ", mode);

  }

  @Override
  public void resetSubsystem() {
    stopDrivetrain();
    zeroSensors();
    configureMotorControllers();
  }

  @Override
  public void zeroSensors() {
    zeroEncoders();
    zeroGyro();
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

  //
  //
  //
  // Gyro stuff
  private int gyroInversion = 1;

  public double getGyroAngle() {
    return gyroInversion * gyro.getAngle();
  }

  public double getAngularVelocity() {
    return gyroInversion * gyro.getRate();
  }

  public void invertGyro(boolean inverted) {
    if (inverted)
      gyroInversion = -1;
    else
      gyroInversion = 1;
  }

  private double gyroCorrectRate = 0.1;

  private double prevAngle;

  public double gyroCorrection() {
    return (getGyroAngle() - prevAngle) * gyroCorrectRate;
  }

  public void zeroGyro() {
    gyro.zeroYaw();
  }

  //
  //
  //
}
