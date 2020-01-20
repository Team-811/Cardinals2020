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

public class Neo extends Subsystem implements ISubsystem{

  private static Neo instance = new Neo();   

  public static Neo getInstance() {
    return instance;
  }

  public Neo()
  {
         
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
      //setDefaultCommand();
  }
}
