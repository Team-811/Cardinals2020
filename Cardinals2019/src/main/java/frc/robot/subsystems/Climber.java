/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This is a subsystem class.  A subsystem interacts with the hardware components on the robot.
 */
public class Climber extends Subsystem implements ISubsystem{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Climber instance = new Climber();
                                                                                                                                      
  public static Climber getInstance() {
      return instance;
  }

  private TalonSRX armMotor;
  private TalonSRX stiltMotor;
  private TalonSRX wheelMotor;

  public Climber()
  {
    armMotor = new TalonSRX(RobotMap.CLIMBER_ARM_MOTOR);
    stiltMotor = new TalonSRX(RobotMap.CLIMBER_STILT_MOTOR);
    wheelMotor = new TalonSRX(RobotMap.CLIMBER_WHEEL_MOTOR); 
  }

  //ClimberMethods

  public void armMotor(double rotation)
  {
    armMotor.set(ControlMode.PercentOutput,rotation);
      
  }

  public void stiltMotor(double rotation)
  {
    stiltMotor.set(ControlMode.PercentOutput,rotation);
      
  }

  public void wheelMotor(double rotation)
  {
    wheelMotor.set(ControlMode.PercentOutput,rotation);
      
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
  public void testSubsystem() {
    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
