/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.Test.NeoTest;


/**
 * This is a subsystem class.  A subsystem interacts with the hardware components on the robot.  
 */

public class Neo extends Subsystem implements ISubsystem{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Neo instance = new Neo();
   

  public static Neo getInstance() {
    return instance;
  }

  private CANSparkMax neo; 
  private CANEncoder encoder;

  public Neo()
  {
      neo = new CANSparkMax(RobotMap.NEO_TEST, MotorType.kBrushless); 
      encoder = neo.getEncoder();     
      zeroSensors();        
  }


  public void NeoTest(double value)
  {     
      neo.set(value);
      SmartDashboard.putNumber("Stick ", value);
  }  

  public void stopNeo()
  {
    neo.stopMotor();
  } 

  @Override
  public void outputSmartdashboard() 
  {
      SmartDashboard.putNumber("Neo Encoder ", encoder.getPosition());
      SmartDashboard.putNumber("Neo Velocity ", encoder.getVelocity());
      
  }

  @Override
  public void zeroSensors() 
  {
    encoder.setPosition(0);
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
      setDefaultCommand(new NeoTest());
  }
}
